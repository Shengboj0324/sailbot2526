module leg_fortran
    implicit none
    
    ! Constants
    real(8), parameter :: PI = 3.14159265358979323846d0
    real(8), parameter :: DEG2RAD = PI / 180.0d0
    real(8), parameter :: RAD2DEG = 180.0d0 / PI
    
    ! Module variables for polar data
    integer :: n_twa = 61  ! Number of TWA values in polar data
    real(8), dimension(61) :: twa_values  ! True wind angles
    real(8), dimension(61) :: boat_speeds  ! Boat speeds at 8 mph wind
    real(8) :: upwind_vmg = 49.3d0   ! Default from polar file
    real(8) :: downwind_vmg = 124.4d0  ! Default from polar file
    logical :: polar_loaded = .false.
    
    ! No-sail zones
    real(8), parameter :: UPWIND_ZONE_START = 315.0d0
    real(8), parameter :: UPWIND_ZONE_END = 45.0d0
    real(8), parameter :: DOWNWIND_ZONE_START = 135.0d0
    real(8), parameter :: DOWNWIND_ZONE_END = 225.0d0
    
contains
    
    subroutine load_polar_data()
        ! Initialize polar data with values from test.pol at 3.5 m/s (8 mph)
        integer :: i
        
        print *, "[FORTRAN] Loading polar data for 8 mph wind speed"
        
        ! TWA values from 0 to 180 degrees
        do i = 1, 61
            twa_values(i) = real((i-1) * 3, 8)  ! 0, 3, 6, ..., 180
        end do
        
        ! Boat speeds for 8 mph (3.5 m/s) wind - column 8 from test.pol
        boat_speeds = (/ &
            0.00769d0, 0.01973d0, 0.13075d0, 1.24982d0, 2.13901d0, &
            2.57333d0, 3.03501d0, 3.5198d0, 4.02126d0, 4.53317d0, &
            5.04967d0, 5.56518d0, 6.07431d0, 6.57188d0, 7.07813d0, &
            7.52545d0, 7.89788d0, 8.23248d0, 8.52616d0, 8.77644d0, &
            8.9813d0, 9.13936d0, 9.24978d0, 9.3123d0, 9.32723d0, &
            9.82972d0, 9.74597d0, 9.74976d0, 9.56986d0, 9.38033d0, &
            9.245d0, 8.98358d0, 8.65359d0, 8.29883d0, 7.77556d0, &
            7.26573d0, 6.81049d0, 6.42228d0, 5.99945d0, 5.59608d0, &
            5.23395d0, 4.92942d0, 4.50882d0, 4.13027d0, 3.83164d0, &
            3.59718d0, 3.4166d0, 3.27477d0, 3.10932d0, 2.95629d0, &
            2.83609d0, 2.74522d0, 2.6839d0, 2.64322d0, 2.58294d0, &
            2.53636d0, 2.50642d0, 2.4849d0, 2.4797d0, 2.48576d0, &
            2.48576d0 /)
        
        print *, "[FORTRAN] Loaded", n_twa, "TWA values"
        print *, "[FORTRAN] TWA range:", twa_values(1), "to", twa_values(n_twa)
        print *, "[FORTRAN] Upwind VMG angle:", upwind_vmg
        print *, "[FORTRAN] Downwind VMG angle:", downwind_vmg
        
        polar_loaded = .true.
    end subroutine load_polar_data
    
    function get_boat_speed(twa) result(speed)
        real(8), intent(in) :: twa
        real(8) :: speed
        real(8) :: abs_twa
        integer :: idx, i
        
        abs_twa = abs(twa)
        
        ! Find nearest TWA index
        idx = 1
        do i = 2, n_twa
            if (abs(twa_values(i) - abs_twa) < abs(twa_values(idx) - abs_twa)) then
                idx = i
            end if
        end do
        
        speed = boat_speeds(idx)
        
        print *, "[FORTRAN] get_boat_speed: TWA =", twa, "-> speed =", speed
    end function get_boat_speed
    
    function normalize_angle(angle) result(normalized)
        real(8), intent(in) :: angle
        real(8) :: normalized
        
        normalized = angle
        do while (normalized > 180.0d0)
            normalized = normalized - 360.0d0
        end do
        do while (normalized < -180.0d0)
            normalized = normalized + 360.0d0
        end do
    end function normalize_angle
    
    subroutine calculate_path(start_lat, start_lon, end_lat, end_lon, &
                             wind_angle, boat_heading, first_maneuver_is_starboard, &
                             waypoints_lat, waypoints_lon, n_waypoints)
        ! Input parameters
        real(8), intent(in) :: start_lat, start_lon  ! Starting position
        real(8), intent(in) :: end_lat, end_lon      ! Ending position
        real(8), intent(in) :: wind_angle             ! Wind angle relative to boat
        real(8), intent(in) :: boat_heading           ! Boat heading
        logical, intent(in) :: first_maneuver_is_starboard  ! First tack/jibe direction
        
        ! Output parameters
        real(8), dimension(10), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints
        
        ! Local variables
        real(8) :: course_angle, course_relative_to_boat
        real(8) :: wind_angle_normalized, target_to_wind_angle
        real(8) :: global_wind_angle
        logical :: upwind_sailing, downwind_sailing
        integer :: i
        
        ! Ensure polar data is loaded
        if (.not. polar_loaded) then
            call load_polar_data()
        end if
        
        print *, "[FORTRAN] ============================================"
        print *, "[FORTRAN] calculate_path called with:"
        print *, "[FORTRAN]   Start:", start_lat, start_lon
        print *, "[FORTRAN]   End:", end_lat, end_lon
        print *, "[FORTRAN]   Wind angle (rel to boat):", wind_angle
        print *, "[FORTRAN]   Boat heading:", boat_heading
        print *, "[FORTRAN]   First maneuver starboard:", first_maneuver_is_starboard
        
        ! Calculate desired course angle (global reference frame)
        course_angle = atan2(end_lon - start_lon, end_lat - start_lat) * RAD2DEG
        print *, "[FORTRAN] Course angle (global):", course_angle
        
        ! Calculate angle between course and boat heading (relative to boat)
        course_relative_to_boat = mod(course_angle - boat_heading, 360.0d0)
        if (course_relative_to_boat > 180.0d0) then
            course_relative_to_boat = course_relative_to_boat - 360.0d0
        end if
        print *, "[FORTRAN] Course relative to boat:", course_relative_to_boat
        
        ! Normalize wind angle to 0-360 range
        wind_angle_normalized = mod(wind_angle, 360.0d0)
        print *, "[FORTRAN] Wind angle normalized:", wind_angle_normalized
        
        ! Determine if upwind or downwind sailing
        upwind_sailing = (wind_angle_normalized >= UPWIND_ZONE_START .or. &
                         wind_angle_normalized <= UPWIND_ZONE_END)
        downwind_sailing = (wind_angle_normalized >= DOWNWIND_ZONE_START .and. &
                           wind_angle_normalized <= DOWNWIND_ZONE_END)
        
        print *, "[FORTRAN] Upwind sailing:", upwind_sailing
        print *, "[FORTRAN] Downwind sailing:", downwind_sailing
        
        ! Calculate angle between desired course and wind
        target_to_wind_angle = mod(course_relative_to_boat - wind_angle_normalized, 360.0d0)
        if (target_to_wind_angle > 180.0d0) then
            target_to_wind_angle = target_to_wind_angle - 360.0d0
        end if
        print *, "[FORTRAN] Target to wind angle:", target_to_wind_angle
        
        ! Convert wind to global reference frame
        global_wind_angle = mod(boat_heading + wind_angle_normalized, 360.0d0)
        print *, "[FORTRAN] Global wind angle:", global_wind_angle
        
        ! Determine if we need to tack or jibe
        if (upwind_sailing .and. abs(target_to_wind_angle) < upwind_vmg) then
            print *, "[FORTRAN] Need to TACK (sailing too close to wind)"
            call calculate_upwind_path(start_lat, start_lon, end_lat, end_lon, &
                                     global_wind_angle, boat_heading, first_maneuver_is_starboard, &
                                     waypoints_lat, waypoints_lon, n_waypoints)
        else if (downwind_sailing .and. abs(target_to_wind_angle) > downwind_vmg) then
            print *, "[FORTRAN] Need to JIBE (sailing too downwind)"
            call calculate_downwind_path(start_lat, start_lon, end_lat, end_lon, &
                                       global_wind_angle, boat_heading, first_maneuver_is_starboard, &
                                       waypoints_lat, waypoints_lon, n_waypoints)
        else
            print *, "[FORTRAN] Direct path possible"
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
        end if
        
        print *, "[FORTRAN] Path calculation complete. Waypoints:", n_waypoints
        do i = 1, n_waypoints
            print *, "[FORTRAN]   Waypoint", i, ":", waypoints_lat(i), waypoints_lon(i)
        end do
        print *, "[FORTRAN] ============================================"
        
    end subroutine calculate_path
    
    subroutine calculate_upwind_path(start_lat, start_lon, end_lat, end_lon, &
                                   global_wind_angle, boat_heading, first_maneuver_is_starboard, &
                                   waypoints_lat, waypoints_lon, n_waypoints)
        real(8), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(8), intent(in) :: global_wind_angle, boat_heading
        logical, intent(in) :: first_maneuver_is_starboard
        real(8), dimension(10), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints
        
        ! Local variables
        real(8) :: target_x, target_y, target_dist
        real(8) :: k_angle, j_angle  ! Starboard and port tack angles
        real(8) :: k_x, k_y, j_x, j_y  ! Unit vectors
        real(8) :: leg1_x, leg1_y, leg2_x, leg2_y
        real(8) :: det, det_scalar1, scalar1
        real(8) :: tack_lat, tack_lon
        
        print *, "[FORTRAN] Calculating upwind (tacking) path"
        
        ! Vector from start to end
        target_x = end_lon - start_lon
        target_y = end_lat - start_lat
        target_dist = sqrt(target_x**2 + target_y**2)
        
        print *, "[FORTRAN] Target vector:", target_x, target_y, "dist:", target_dist
        
        ! Calculate tacking angles (global frame)
        ! Wind + 180 +/- upwind_vmg
        k_angle = global_wind_angle + 180.0d0 + upwind_vmg  ! Starboard tack
        j_angle = global_wind_angle + 180.0d0 - upwind_vmg  ! Port tack
        
        print *, "[FORTRAN] Starboard tack angle:", k_angle
        print *, "[FORTRAN] Port tack angle:", j_angle
        
        ! Convert to unit vectors
        k_x = cos(k_angle * DEG2RAD)
        k_y = sin(k_angle * DEG2RAD)
        j_x = cos(j_angle * DEG2RAD)
        j_y = sin(j_angle * DEG2RAD)
        
        ! Choose leg vectors based on first maneuver
        if (first_maneuver_is_starboard) then
            leg1_x = k_x
            leg1_y = k_y
            leg2_x = j_x
            leg2_y = j_y
            print *, "[FORTRAN] First leg: STARBOARD"
        else
            leg1_x = j_x
            leg1_y = j_y
            leg2_x = k_x
            leg2_y = k_y
            print *, "[FORTRAN] First leg: PORT"
        end if
        
        ! Solve system: target = scalar1 * leg1 + scalar2 * leg2
        det = leg1_x * leg2_y - leg2_x * leg1_y
        
        if (abs(det) < 1.0d-9) then
            print *, "[FORTRAN] WARNING: Collinear vectors, using direct path"
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            return
        end if
        
        det_scalar1 = target_x * leg2_y - leg2_x * target_y
        scalar1 = det_scalar1 / det
        
        print *, "[FORTRAN] Determinant:", det
        print *, "[FORTRAN] Scalar1:", scalar1
        
        ! Calculate tack point
        tack_lon = start_lon + leg1_x * scalar1
        tack_lat = start_lat + leg1_y * scalar1
        
        print *, "[FORTRAN] Tack point:", tack_lat, tack_lon
        
        ! Return waypoints
        waypoints_lat(1) = tack_lat
        waypoints_lon(1) = tack_lon
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        
    end subroutine calculate_upwind_path
    
    subroutine calculate_downwind_path(start_lat, start_lon, end_lat, end_lon, &
                                     global_wind_angle, boat_heading, first_maneuver_is_starboard, &
                                     waypoints_lat, waypoints_lon, n_waypoints)
        real(8), intent(in) :: start_lat, start_lon, end_lat, end_lon
        real(8), intent(in) :: global_wind_angle, boat_heading
        logical, intent(in) :: first_maneuver_is_starboard
        real(8), dimension(10), intent(out) :: waypoints_lat, waypoints_lon
        integer, intent(out) :: n_waypoints
        
        ! Local variables
        real(8) :: target_x, target_y, target_dist
        real(8) :: k_angle, j_angle  ! Starboard and port jibe angles
        real(8) :: k_x, k_y, j_x, j_y  ! Unit vectors
        real(8) :: leg1_x, leg1_y, leg2_x, leg2_y
        real(8) :: det, det_scalar1, scalar1
        real(8) :: jibe_lat, jibe_lon
        
        print *, "[FORTRAN] Calculating downwind (jibing) path"
        
        ! Vector from start to end
        target_x = end_lon - start_lon
        target_y = end_lat - start_lat
        target_dist = sqrt(target_x**2 + target_y**2)
        
        print *, "[FORTRAN] Target vector:", target_x, target_y, "dist:", target_dist
        
        ! Calculate jibing angles (global frame)
        ! Wind + 180 +/- downwind_vmg
        k_angle = global_wind_angle + 180.0d0 + downwind_vmg  ! Starboard jibe
        j_angle = global_wind_angle + 180.0d0 - downwind_vmg  ! Port jibe
        
        print *, "[FORTRAN] Starboard jibe angle:", k_angle
        print *, "[FORTRAN] Port jibe angle:", j_angle
        
        ! Convert to unit vectors
        k_x = cos(k_angle * DEG2RAD)
        k_y = sin(k_angle * DEG2RAD)
        j_x = cos(j_angle * DEG2RAD)
        j_y = sin(j_angle * DEG2RAD)
        
        ! Choose leg vectors based on first maneuver
        if (first_maneuver_is_starboard) then
            leg1_x = k_x
            leg1_y = k_y
            leg2_x = j_x
            leg2_y = j_y
            print *, "[FORTRAN] First leg: STARBOARD"
        else
            leg1_x = j_x
            leg1_y = j_y
            leg2_x = k_x
            leg2_y = k_y
            print *, "[FORTRAN] First leg: PORT"
        end if
        
        ! Solve system: target = scalar1 * leg1 + scalar2 * leg2
        det = leg1_x * leg2_y - leg2_x * leg1_y
        
        if (abs(det) < 1.0d-9) then
            print *, "[FORTRAN] WARNING: Collinear vectors, using direct path"
            waypoints_lat(1) = end_lat
            waypoints_lon(1) = end_lon
            n_waypoints = 1
            return
        end if
        
        det_scalar1 = target_x * leg2_y - leg2_x * target_y
        scalar1 = det_scalar1 / det
        
        print *, "[FORTRAN] Determinant:", det
        print *, "[FORTRAN] Scalar1:", scalar1
        
        ! Calculate jibe point
        jibe_lon = start_lon + leg1_x * scalar1
        jibe_lat = start_lat + leg1_y * scalar1
        
        print *, "[FORTRAN] Jibe point:", jibe_lat, jibe_lon
        
        ! Return waypoints
        waypoints_lat(1) = jibe_lat
        waypoints_lon(1) = jibe_lon
        waypoints_lat(2) = end_lat
        waypoints_lon(2) = end_lon
        n_waypoints = 2
        
    end subroutine calculate_downwind_path

end module leg_fortran