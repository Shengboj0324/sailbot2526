!> Modern Fortran 2018+ implementation of sailboat path planning
!> Uses ISO_C_BINDING for clean Python interface and modern Fortran features
module leg_modern
    use, intrinsic :: iso_fortran_env, only: real64, int32, error_unit
    use, intrinsic :: iso_c_binding
    use, intrinsic :: ieee_arithmetic
    implicit none
    private
    
    ! Public API
    public :: c_calculate_path, c_initialize_polar_data
    
    ! High precision constants
    real(real64), parameter :: PI = 4.0_real64 * atan(1.0_real64)
    real(real64), parameter :: DEG2RAD = PI / 180.0_real64
    real(real64), parameter :: RAD2DEG = 180.0_real64 / PI
    real(real64), parameter :: EPSILON = 1.0e-9_real64
    
    ! Optimization parameters
    integer(int32), parameter :: MAX_WAYPOINTS = 10
    integer(int32), parameter :: N_TWA_POINTS = 61
    
    ! Error codes
    enum, bind(c)
        enumerator :: SUCCESS = 0
        enumerator :: ERROR_INVALID_INPUT = -1
        enumerator :: ERROR_COLLINEAR_VECTORS = -2
        enumerator :: ERROR_POLAR_NOT_LOADED = -3
    end enum
    
    !> Modern derived type for polar data with type-bound procedures
    type :: polar_data_t
        real(real64), dimension(N_TWA_POINTS) :: twa_values
        real(real64), dimension(N_TWA_POINTS) :: boat_speeds
        real(real64) :: upwind_vmg
        real(real64) :: downwind_vmg
        logical :: initialized = .false.
    contains
        procedure :: get_boat_speed => polar_get_boat_speed
        procedure :: load_fixed_data => polar_load_fixed_data
    end type polar_data_t
    
    ! Module variable for polar data
    type(polar_data_t), save :: polar_data
    
    ! No-sail zones (in degrees)
    real(real64), parameter :: UPWIND_ZONE_START = 315.0_real64
    real(real64), parameter :: UPWIND_ZONE_END = 45.0_real64
    real(real64), parameter :: DOWNWIND_ZONE_START = 135.0_real64
    real(real64), parameter :: DOWNWIND_ZONE_END = 225.0_real64
    
contains
    
    !> Initialize polar data for 8 mph wind speed
    function c_initialize_polar_data() bind(c, name="initialize_polar_data") result(status)
        integer(c_int) :: status
        
        call polar_data%load_fixed_data()
        status = SUCCESS
    end function c_initialize_polar_data
    
    !> Load fixed polar data for 8 mph wind
    subroutine polar_load_fixed_data(this)
        class(polar_data_t), intent(inout) :: this
        integer :: i
        
        ! Generate TWA values
        do concurrent (i = 1:N_TWA_POINTS)
            this%twa_values(i) = real((i-1) * 3, real64)
        end do
        
        ! Boat speeds for 8 mph (3.5 m/s) wind
        this%boat_speeds = [ &
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
        
        this%upwind_vmg = 49.3_real64
        this%downwind_vmg = 124.4_real64
        this%initialized = .true.
    end subroutine polar_load_fixed_data
    
    !> Get boat speed for given true wind angle using vectorized search
    pure function polar_get_boat_speed(this, twa) result(speed)
        class(polar_data_t), intent(in) :: this
        real(real64), intent(in) :: twa
        real(real64) :: speed
        real(real64) :: abs_twa
        integer :: idx
        
        abs_twa = abs(twa)
        
        ! Vectorized minimum search
        idx = minloc(abs(this%twa_values - abs_twa), dim=1)
        speed = this%boat_speeds(idx)
    end function polar_get_boat_speed
    
    !> Main path calculation function with C interface
    function c_calculate_path(start_lat, start_lon, end_lat, end_lon, &
                             wind_angle, boat_heading, first_maneuver_is_starboard, &
                             waypoints_lat, waypoints_lon, n_waypoints) &
                             bind(c, name="calculate_path") result(status)
        real(c_double), value, intent(in) :: start_lat, start_lon
        real(c_double), value, intent(in) :: end_lat, end_lon
        real(c_double), value, intent(in) :: wind_angle
        real(c_double), value, intent(in) :: boat_heading
        logical(c_bool), value, intent(in) :: first_maneuver_is_starboard
        real(c_double), dimension(MAX_WAYPOINTS), intent(out) :: waypoints_lat, waypoints_lon
        integer(c_int), intent(out) :: n_waypoints
        integer(c_int) :: status
        
        ! Validate inputs
        if (.not. polar_data%initialized) then
            status = ERROR_POLAR_NOT_LOADED
            n_waypoints = 0
            return
        end if
        
        if (.not. ieee_is_finite(start_lat) .or. .not. ieee_is_finite(start_lon) .or. &
            .not. ieee_is_finite(end_lat) .or. .not. ieee_is_finite(end_lon)) then
            status = ERROR_INVALID_INPUT
            n_waypoints = 0
            return
        end if
        
        ! Call internal calculation
        call calculate_path_internal(start_lat, start_lon, end_lat, end_lon, &
                                   wind_angle, boat_heading, &
                                   logical(first_maneuver_is_starboard), &
                                   waypoints_lat, waypoints_lon, n_waypoints, status)
    end function c_calculate_path
    
    !> Internal path calculation with modern Fortran features
    subroutine calculate_path_internal(start_lat, start_lon, end_lat, end_lon, &
                                      wind_angle, boat_heading, first_maneuver_is_starboard, &
                                      waypoints_lat, waypoints_lon, n_waypoints, status)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: wind_angle, boat_heading
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        
        real(real64) :: course_angle, course_relative_to_boat
        real(real64) :: wind_angle_normalized, target_to_wind_angle
        real(real64) :: global_wind_angle
        logical :: upwind_sailing, downwind_sailing
        
        ! Calculate course angle using atan2
        course_angle = atan2(end_lon - start_lon, end_lat - start_lat) * RAD2DEG
        
        ! Normalize angles using modulo arithmetic
        course_relative_to_boat = modulo(course_angle - boat_heading, 360.0_real64)
        if (course_relative_to_boat > 180.0_real64) course_relative_to_boat = course_relative_to_boat - 360.0_real64
        
        wind_angle_normalized = modulo(wind_angle, 360.0_real64)
        
        ! Determine sailing mode
        associate (wan => wind_angle_normalized)
            upwind_sailing = (wan >= UPWIND_ZONE_START .or. wan <= UPWIND_ZONE_END)
            downwind_sailing = (wan >= DOWNWIND_ZONE_START .and. wan <= DOWNWIND_ZONE_END)
        end associate
        
        ! Calculate target to wind angle
        target_to_wind_angle = modulo(course_relative_to_boat - wind_angle_normalized, 360.0_real64)
        if (target_to_wind_angle > 180.0_real64) target_to_wind_angle = target_to_wind_angle - 360.0_real64
        
        global_wind_angle = modulo(boat_heading + wind_angle_normalized, 360.0_real64)
        
        ! Determine path type and calculate
        if (upwind_sailing .and. abs(target_to_wind_angle) < polar_data%upwind_vmg) then
            call calculate_tacking_path(start_lat, start_lon, end_lat, end_lon, &
                                      global_wind_angle, first_maneuver_is_starboard, &
                                      waypoints_lat, waypoints_lon, n_waypoints, status)
        else if (downwind_sailing .and. abs(target_to_wind_angle) > polar_data%downwind_vmg) then
            call calculate_jibing_path(start_lat, start_lon, end_lat, end_lon, &
                                     global_wind_angle, first_maneuver_is_starboard, &
                                     waypoints_lat, waypoints_lon, n_waypoints, status)
        else
            ! Direct path
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            status = SUCCESS
        end if
    end subroutine calculate_path_internal
    
    !> Calculate optimal tacking path using modern vector operations
    subroutine calculate_tacking_path(start_lat, start_lon, end_lat, end_lon, &
                                    global_wind_angle, first_maneuver_is_starboard, &
                                    waypoints_lat, waypoints_lon, n_waypoints, status)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: global_wind_angle
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        
        real(real64) :: target_vec(2), leg1_vec(2), leg2_vec(2)
        real(real64) :: k_angle, j_angle
        real(real64) :: matrix(2,2), det, scalar1
        
        ! Target vector
        target_vec = [end_lon - start_lon, end_lat - start_lat]
        
        ! Calculate tacking angles
        k_angle = global_wind_angle + 180.0_real64 + polar_data%upwind_vmg
        j_angle = global_wind_angle + 180.0_real64 - polar_data%upwind_vmg
        
        ! Choose legs based on first maneuver
        if (first_maneuver_is_starboard) then
            leg1_vec = [cos(k_angle * DEG2RAD), sin(k_angle * DEG2RAD)]
            leg2_vec = [cos(j_angle * DEG2RAD), sin(j_angle * DEG2RAD)]
        else
            leg1_vec = [cos(j_angle * DEG2RAD), sin(j_angle * DEG2RAD)]
            leg2_vec = [cos(k_angle * DEG2RAD), sin(k_angle * DEG2RAD)]
        end if
        
        ! Solve linear system using determinants
        matrix = reshape([leg1_vec, leg2_vec], [2, 2])
        det = matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1)
        
        if (abs(det) < EPSILON) then
            status = ERROR_COLLINEAR_VECTORS
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            return
        end if
        
        ! Cramer's rule for scalar1
        scalar1 = (target_vec(1) * matrix(2,2) - target_vec(2) * matrix(1,2)) / det
        
        ! Calculate tack point
        waypoints_lon(1) = start_lon + leg1_vec(1) * scalar1
        waypoints_lat(1) = start_lat + leg1_vec(2) * scalar1
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        status = SUCCESS
    end subroutine calculate_tacking_path
    
    !> Calculate optimal jibing path
    subroutine calculate_jibing_path(start_lat, start_lon, end_lat, end_lon, &
                                   global_wind_angle, first_maneuver_is_starboard, &
                                   waypoints_lat, waypoints_lon, n_waypoints, status)
        real(real64), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(real64), intent(in) :: global_wind_angle
        logical, intent(in) :: first_maneuver_is_starboard
        real(real64), dimension(:), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints, status
        
        real(real64) :: target_vec(2), leg1_vec(2), leg2_vec(2)
        real(real64) :: k_angle, j_angle
        real(real64) :: matrix(2,2), det, scalar1
        
        ! Target vector
        target_vec = [end_lon - start_lon, end_lat - start_lat]
        
        ! Calculate jibing angles
        k_angle = global_wind_angle + 180.0_real64 + polar_data%downwind_vmg
        j_angle = global_wind_angle + 180.0_real64 - polar_data%downwind_vmg
        
        ! Choose legs based on first maneuver
        if (first_maneuver_is_starboard) then
            leg1_vec = [cos(k_angle * DEG2RAD), sin(k_angle * DEG2RAD)]
            leg2_vec = [cos(j_angle * DEG2RAD), sin(j_angle * DEG2RAD)]
        else
            leg1_vec = [cos(j_angle * DEG2RAD), sin(j_angle * DEG2RAD)]
            leg2_vec = [cos(k_angle * DEG2RAD), sin(k_angle * DEG2RAD)]
        end if
        
        ! Solve linear system
        matrix = reshape([leg1_vec, leg2_vec], [2, 2])
        det = matrix(1,1) * matrix(2,2) - matrix(1,2) * matrix(2,1)
        
        if (abs(det) < EPSILON) then
            status = ERROR_COLLINEAR_VECTORS
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            return
        end if
        
        scalar1 = (target_vec(1) * matrix(2,2) - target_vec(2) * matrix(1,2)) / det
        
        ! Calculate jibe point
        waypoints_lon(1) = start_lon + leg1_vec(1) * scalar1
        waypoints_lat(1) = start_lat + leg1_vec(2) * scalar1
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        status = SUCCESS
    end subroutine calculate_jibing_path

end module leg_modern