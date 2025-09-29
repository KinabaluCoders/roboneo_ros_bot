def load_config():
    """
    Load configuration settings for the robot.
    Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
    """
    return {
        'events_url'            : 'https://pluto.kinabalumakers.dev/roboneo_2025/events', # will be provided by event Instructor
        'secret_key'            : 'YOUR_SECRET_KEY', # will be provided by event Instructor. Dont share with anyone
        'robot_id'              : 'YOUR_ROBOT_ID', # will be provided by event Instructor
        'tournament_round'      : 0, # 0 for TESTING,  1 - 3 for TOURNAMENT rounds

        # Ultrasonic distance to consider as obstacle
        # Example within 0 to 17.0 cm
        'detected_distance'     : 0.0, 

        # Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
        # Example 0.5 = 127, -0.5 = -127, 1.0 = 255, -1.0 = -255
        'forward_speed'         : 1.0,  # Forward speed
        'reverse_speed'         : -1.0, # Reverse speed

        # Speed in range -1.0 to 1.0 is mapped to -255 to 255 for motor control.
        'turn_speed'            : 1.0,  # Turning speed

        # Time durations for various maneuvers
        # Example 0.7 seconds to turn ~90 degrees
        'turn_90deg_duration'   : 1.0,  # seconds to turn ~90 degrees
        'turn_180deg_duration'  : 1.0,  # seconds to turn ~180 degrees example turn from right to left

        'floor_color_arena_border'      : 'Red', # the robot will avoid this color line, case sensitive

        ### **************** Floor Color Detection Mode Settings ****************
        # Zone 1 color line to detect floor line color for points
        # Example: Blue line is 5 points
        ### ****************************************************************
        #      
        'floor_color_zone_one_color_0'  : 'Blue', # detect color to get points, case sensitive
        'floor_color_zone_one_color_1'  : 'Green', # detect color to get points, case sensitive
        
    
        ### **************** Pillar Detection Mode Settings ****************
        # Zone 2 color line to switch to Pillar Detection Mode
        ### ****************************************************************

        'floor_color_zone_two_entrance' : 'Black', # color of zone two, Pillar detection mode, case sensitive

        # This is used on Pillar scanning (Zone 2)
        # Example speed 0.5 mapped to 127.5
        'pillar_scan_turn_speed'        : 1.0, # Slow turn speed for pillar scanning

        # Time(seconds) durations for pillar scanning maneuvers
        # Example 2.7 (seconds) for a full 360-degree rotation
        'pillar_scan_duration'          : 3.0,

        # Cooldown(seconds) periods to prevent rapid state changes
        'scan_cooldown'                 : 5.0,

        # LiDAR distance thresholds for pillar detection (in cm)
        'min_lidar_distance'            : 100.0, # minimum distance to consider as pillar
        'max_lidar_distance'            : 100.0, # maximum distance to consider as pillar
    }
    