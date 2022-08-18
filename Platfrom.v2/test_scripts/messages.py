MSG_STOP_DRIVING = 'ST0+00000+00000E'    # message to stop robot
MSG_TURN_OFF_LIGHT = 'SU--00000000000E'    # message to turn off lighting
direction_of_movements_robot = {
                                    'LEFT': 'ST0-00300+00300E', 'RIGHT': 'ST0+00300-00300E',
                                    'FORWARD': 'ST0+00200+00200E', 'STOP': 'ST0+00000+00000E',
                                    'BACK': 'ST0-00200-00200E'
                                }

