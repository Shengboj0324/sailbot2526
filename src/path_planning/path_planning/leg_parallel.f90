!> Ultra-high-performance Fortran 2023 implementation with OpenMP
!> Optimized for batch processing and parallel execution
module leg_parallel
    use, intrinsic :: iso_fortran_env, only: real64, int32, int64, error_unit
    use, intrinsic :: iso_c_binding
    use, intrinsic :: ieee_arithmetic
    use omp_lib
    implicit none
    private
    
    ! Public API
    public :: c_calculate_path_batch, c_initialize_system, c_cleanup_system
    public :: c_calculate_path_single
    
    ! Constants with extended precision
    real(real64), parameter :: PI = acos(-1.0_real64)
    real(real64), parameter :: DEG2RAD = PI / 180.0_real64
    real(real64), parameter :: RAD2DEG = 180.0_real64 / PI
    real(real64), parameter :: EPSILON = 1.0e-9_real64
    
    ! Configuration
    integer(int32), parameter :: MAX_WAYPOINTS = 10
    integer(int32), parameter :: N_TWA_POINTS = 61
    integer(int32), parameter :: CACHE_LINE_SIZE = 64
    
    ! Error codes
    enum, bind(c)
        enumerator :: SUCCESS = 0
        enumerator :: ERROR_INVALID_INPUT = -1
        enumerator :: ERROR_COLLINEAR_VECTORS = -2
        enumerator :: ERROR_SYSTEM_NOT_INITIALIZED = -3
        enumerator :: ERROR_ALLOCATION_FAILED = -4
    end enum
    
    !> Optimized polar data with cache-aligned storage
    type :: polar_data_t
        real(real64), dimension(N_TWA_POINTS) :: twa_values
        real(real64), dimension(N_TWA_POINTS) :: boat_speeds
        real(real64) :: upwind_vmg
        real(real64) :: downwind_vmg
        ! Padding for cache alignment
        real(real64) :: padding(6)
    end type polar_data_t
    
    !> Thread-local workspace for calculations
    type :: workspace_t
        real(real64), dimension(2,2) :: matrix
        real(real64), dimension(2) :: target_vec
        real(real64), dimension(2) :: leg1_vec
        real(real64), dimension(2) :: leg2_vec
        real(real64) :: scratch(8)  ! Cache-aligned scratch space
    end type workspace_t
    
    ! Module variables
    type(polar_data_t), allocatable, save :: polar_data
    type(workspace_t), allocatable, dimension(:), save :: workspaces
    logical, save :: system_initialized = .false.
    integer(int32), save :: num_threads = 1
    
    ! Zone boundaries (compile-time constants for optimization)
    real(real64), parameter :: UPWIND_ZONE_START = 315.0_real64
    real(real64), parameter :: UPWIND_ZONE_END = 45.0_real64
    real(real64), parameter :: DOWNWIND_ZONE_START = 135.0_real64
    real(real64), parameter :: DOWNWIND_ZONE_END = 225.0_real64
    
contains
    
    !> Initialize the system with optimal thread configuration
    function c_initialize_system() bind(c, name="initialize_system") result(status)
        integer(c_int) :: status
        integer :: ierr
        
        if (system_initialized) then
            status = SUCCESS
            return
        end if
        
        ! Determine optimal thread count
        !$omp parallel
        !$omp single
        num_threads = omp_get_num_threads()
        !$omp end single
        !$omp end parallel
        
        ! Allocate polar data
        allocate(polar_data, stat=ierr)
        if (ierr /= 0) then
            status = ERROR_ALLOCATION_FAILED
            return
        end if
        
        ! Allocate thread-local workspaces
        allocate(workspaces(num_threads), stat=ierr)
        if (ierr /= 0) then
            status = ERROR_ALLOCATION_FAILED
            return
        end if
        
        ! Initialize polar data
        call initialize_polar_data()
        
        system_initialized = .true.
        status = SUCCESS
    end function c_initialize_system
    
    !> Cleanup system resources
    function c_cleanup_system() bind(c, name="cleanup_system") result(status)
        integer(c_int) :: status
        
        if (allocated(polar_data)) deallocate(polar_data)
        if (allocated(workspaces)) deallocate(workspaces)
        system_initialized = .false.
        status = SUCCESS
    end function c_cleanup_system
    
    !> Initialize polar data with vectorized operations
    subroutine initialize_polar_data()
        integer :: i
        
        ! Vectorized TWA generation
        !$omp simd
        do i = 1, N_TWA_POINTS
            polar_data%twa_values(i) = real((i-1) * 3, real64)
        end do
        
        ! Boat speeds for 8 mph wind (optimized memory layout)
        polar_data%boat_speeds = [ &
            0.00769_real64, 0.01973_real64, 0.13075_real64, 1.24982_real64, 2.13901_real64, &
            2.57333_real64, 3.03501_real64, 3.5198_real64, 4.02126_real64, 4.53317_real64, &
            5.04967_real64, 5.56518_real64, 6.07431_real64, 6.57188_real64, 7.07813_real64, &
            7.52545_real64, 7.89788_real64, 8.23248_real64, 8.52616_real64, 8.77644_real64, &
            8.9813_real64, 9.13936_real64, 9.24978_real64, 9.3123_real64, 9.32723_real64, &
            9.82972_real64, 9.74597_real64, 9.74976_real64, 9.56986_real64, 9.38033_real64, &
            9.245_real64, 8.98358_real64, 8.65359_real64, 8.29883_real64, 7.77556_real64, &
            7.26573_real64, 6.81049_real64, 6.42228_real64, 5.99945_real64, 5.59608_real64, &
            5.23395_real64, 4.92942_real64, 4.50882_real64, 4.13027_real64, 3.83164_real64, &
            3.59718_real64, 3.4166_real64, 3.27477_real64, 3.10932_real64, 2.95629_real64, &
            2.83609_real64, 2.74522_real64, 2.6839_real64, 2.64322_real64, 2.58294_real64, &
            2.53636_real64, 2.50642_real64, 2.4849_real64, 2.4797_real64, 2.48576_real64, &
            2.48576_real64 ]
        
        polar_data%upwind_vmg = 49.3_real64
        polar_data%downwind_vmg = 124.4_real64
    end subroutine initialize_polar_data
    
    !> SIMD-optimized boat speed lookup
    pure function get_boat_speed_simd(twa) result(speed)
        real(real64), intent(in) :: twa
        real(real64) :: speed
        real(real64) :: abs_twa, min_diff
        integer :: idx, i
        
        abs_twa = abs(twa)
        idx = 1
        min_diff = abs(polar_data%twa_values(1) - abs_twa)
        
        ! Vectorized search
        !$omp simd reduction(min:min_diff)
        do i = 2, N_TWA_POINTS
            if (abs(polar_data%twa_values(i) - abs_twa) < min_diff) then
                min_diff = abs(polar_data%twa_values(i) - abs_twa)
                idx = i
            end if
        end do
        
        speed = polar_data%boat_speeds(idx)
    end function get_boat_speed_simd
    
    !> Single path calculation with optimizations
    function c_calculate_path_single(start_lat, start_lon, end_lat, end_lon, &
                                   wind_angle, boat_heading, first_maneuver_is_starboard, &
                                   waypoints_lat, waypoints_lon, n_waypoints) &
                                   bind(c, name="calculate_path_single") result(status)
        real(c_double), value, intent(in) :: start_lat, start_lon
        real(c_double), value, intent(in) :: end_lat, end_lon
        real(c_double), value, intent(in) :: wind_angle
        real(c_double), value, intent(in) :: boat_heading
        logical(c_bool), value, intent(in) :: first_maneuver_is_starboard
        real(c_double), dimension(MAX_WAYPOINTS), intent(out) :: waypoints_lat, waypoints_lon
        integer(c_int), intent(out) :: n_waypoints
        integer(c_int) :: status
        
        integer :: thread_id
        
        if (.not. system_initialized) then
            status = ERROR_SYSTEM_NOT_INITIALIZED
            n_waypoints = 0
            return
        end if
        
        ! Get thread ID for workspace
        thread_id = omp_get_thread_num() + 1
        
        call calculate_path_optimized( &
            start_lat, start_lon, end_lat, end_lon, &
            wind_angle, boat_heading, logical(first_maneuver_is_starboard), &
            waypoints_lat, waypoints_lon, n_waypoints, status, &
            workspaces(thread_id))
    end function c_calculate_path_single
    
    !> Batch processing of multiple paths in parallel
    function c_calculate_path_batch(n_paths, &
                                  start_lats, start_lons, end_lats, end_lons, &
                                  wind_angles, boat_headings, first_maneuvers, &
                                  all_waypoints_lat, all_waypoints_lon, all_n_waypoints, &
                                  statuses) bind(c, name="calculate_path_batch") result(overall_status)
        integer(c_int), value, intent(in) :: n_paths
        real(c_double), dimension(n_paths), intent(in) :: start_lats, start_lons
        real(c_double), dimension(n_paths), intent(in) :: end_lats, end_lons
        real(c_double), dimension(n_paths), intent(in) :: wind_angles, boat_headings
        logical(c_bool), dimension(n_paths), intent(in) :: first_maneuvers
        real(c_double), dimension(MAX_WAYPOINTS, n_paths), intent(out) :: all_waypoints_lat, all_waypoints_lon
        integer(c_int), dimension(n_paths), intent(out) :: all_n_waypoints
        integer(c_int), dimension(n_paths), intent(out) :: statuses
        integer(c_int) :: overall_status
        
        integer :: i
        
        if (.not. system_initialized) then
            overall_status = ERROR_SYSTEM_NOT_INITIALIZED
            statuses = ERROR_SYSTEM_NOT_INITIALIZED
            all_n_waypoints = 0
            return
        end if
        
        ! Parallel processing of paths
        !$omp parallel do schedule(dynamic) private(i)
        do i = 1, n_paths
            call calculate_path_optimized( &
                start_lats(i), start_lons(i), end_lats(i), end_lons(i), &
                wind_angles(i), boat_headings(i), logical(first_maneuvers(i)), &
                all_waypoints_lat(:,i), all_waypoints_lon(:,i), &
                all_n_waypoints(i), statuses(i), &
                workspaces(omp_get_thread_num() + 1))
        end do
        !$omp end parallel do
        
        overall_status = SUCCESS
    end function c_calculate_path_batch
    
    !> Optimized path calculation core
    subroutine calculate_path_optimized(start_lat, start_lon, end_lat, end_lon, &
                                       wind_angle, boat_heading, first_maneuver_is_starboard, &
                                       waypoints_lat, waypoints_lon, n_waypoints, status, &
                                       workspace)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: wind_angle, boat_heading
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        type(workspace_t), intent(inout) :: workspace
        
        real(real64) :: course_angle, course_relative_to_boat
        real(real64) :: wind_angle_normalized, target_to_wind_angle
        real(real64) :: global_wind_angle
        logical :: upwind_sailing, downwind_sailing
        
        ! Fast path for same start/end
        if (abs(start_lat - end_lat) < EPSILON .and. abs(start_lon - end_lon) < EPSILON) then
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            status = SUCCESS
            return
        end if
        
        ! Optimized angle calculations
        course_angle = atan2(end_lon - start_lon, end_lat - start_lat) * RAD2DEG
        course_relative_to_boat = modulo(course_angle - boat_heading, 360.0_real64)
        if (course_relative_to_boat > 180.0_real64) course_relative_to_boat = course_relative_to_boat - 360.0_real64
        
        wind_angle_normalized = modulo(wind_angle, 360.0_real64)
        
        ! Branchless sailing mode determination
        upwind_sailing = (wind_angle_normalized >= UPWIND_ZONE_START) .or. &
                        (wind_angle_normalized <= UPWIND_ZONE_END)
        downwind_sailing = (wind_angle_normalized >= DOWNWIND_ZONE_START) .and. &
                          (wind_angle_normalized <= DOWNWIND_ZONE_END)
        
        target_to_wind_angle = modulo(course_relative_to_boat - wind_angle_normalized, 360.0_real64)
        if (target_to_wind_angle > 180.0_real64) target_to_wind_angle = target_to_wind_angle - 360.0_real64
        
        global_wind_angle = modulo(boat_heading + wind_angle_normalized, 360.0_real64)
        
        ! Path determination with optimized branching
        if (upwind_sailing .and. abs(target_to_wind_angle) < polar_data%upwind_vmg) then
            call calculate_tacking_optimized(start_lat, start_lon, end_lat, end_lon, &
                                           global_wind_angle, first_maneuver_is_starboard, &
                                           waypoints_lat, waypoints_lon, n_waypoints, status, workspace)
        else if (downwind_sailing .and. abs(target_to_wind_angle) > polar_data%downwind_vmg) then
            call calculate_jibing_optimized(start_lat, start_lon, end_lat, end_lon, &
                                          global_wind_angle, first_maneuver_is_starboard, &
                                          waypoints_lat, waypoints_lon, n_waypoints, status, workspace)
        else
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            status = SUCCESS
        end if
    end subroutine calculate_path_optimized
    
    !> Optimized tacking calculation using workspace
    subroutine calculate_tacking_optimized(start_lat, start_lon, end_lat, end_lon, &
                                         global_wind_angle, first_maneuver_is_starboard, &
                                         waypoints_lat, waypoints_lon, n_waypoints, status, &
                                         workspace)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: global_wind_angle
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        type(workspace_t), intent(inout) :: workspace
        
        real(real64) :: k_angle, j_angle, det, scalar1
        
        ! Use workspace for calculations
        workspace%target_vec = [end_lon - start_lon, end_lat - start_lat]
        
        k_angle = (global_wind_angle + 180.0_real64 + polar_data%upwind_vmg) * DEG2RAD
        j_angle = (global_wind_angle + 180.0_real64 - polar_data%upwind_vmg) * DEG2RAD
        
        if (first_maneuver_is_starboard) then
            workspace%leg1_vec = [cos(k_angle), sin(k_angle)]
            workspace%leg2_vec = [cos(j_angle), sin(j_angle)]
        else
            workspace%leg1_vec = [cos(j_angle), sin(j_angle)]
            workspace%leg2_vec = [cos(k_angle), sin(k_angle)]
        end if
        
        ! Optimized determinant calculation
        det = workspace%leg1_vec(1) * workspace%leg2_vec(2) - workspace%leg1_vec(2) * workspace%leg2_vec(1)
        
        if (abs(det) < EPSILON) then
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            status = ERROR_COLLINEAR_VECTORS
            return
        end if
        
        ! Cramer's rule
        scalar1 = (workspace%target_vec(1) * workspace%leg2_vec(2) - &
                  workspace%target_vec(2) * workspace%leg2_vec(1)) / det
        
        waypoints_lon(1) = start_lon + workspace%leg1_vec(1) * scalar1
        waypoints_lat(1) = start_lat + workspace%leg1_vec(2) * scalar1
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        status = SUCCESS
    end subroutine calculate_tacking_optimized
    
    !> Optimized jibing calculation
    subroutine calculate_jibing_optimized(start_lat, start_lon, end_lat, end_lon, &
                                        global_wind_angle, first_maneuver_is_starboard, &
                                        waypoints_lat, waypoints_lon, n_waypoints, status, &
                                        workspace)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: global_wind_angle
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        type(workspace_t), intent(inout) :: workspace
        
        real(real64) :: k_angle, j_angle, det, scalar1
        
        workspace%target_vec = [end_lon - start_lon, end_lat - start_lat]
        
        k_angle = (global_wind_angle + 180.0_real64 + polar_data%downwind_vmg) * DEG2RAD
        j_angle = (global_wind_angle + 180.0_real64 - polar_data%downwind_vmg) * DEG2RAD
        
        if (first_maneuver_is_starboard) then
            workspace%leg1_vec = [cos(k_angle), sin(k_angle)]
            workspace%leg2_vec = [cos(j_angle), sin(j_angle)]
        else
            workspace%leg1_vec = [cos(j_angle), sin(j_angle)]
            workspace%leg2_vec = [cos(k_angle), sin(k_angle)]
        end if
        
        det = workspace%leg1_vec(1) * workspace%leg2_vec(2) - workspace%leg1_vec(2) * workspace%leg2_vec(1)
        
        if (abs(det) < EPSILON) then
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            status = ERROR_COLLINEAR_VECTORS
            return
        end if
        
        scalar1 = (workspace%target_vec(1) * workspace%leg2_vec(2) - &
                  workspace%target_vec(2) * workspace%leg2_vec(1)) / det
        
        waypoints_lon(1) = start_lon + workspace%leg1_vec(1) * scalar1
        waypoints_lat(1) = start_lat + workspace%leg1_vec(2) * scalar1
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        status = SUCCESS
    end subroutine calculate_jibing_optimized

end module leg_parallel