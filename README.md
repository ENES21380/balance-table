                dx = current_x - center_x
                dy = current_y - center_y
                sector_pwm = 180
                rest_pwm = 75
                if abs(dx) < 10 and abs(dy) < 10:
                    pwm_a = pwm_b = pwm_c = pwm_d = rest_pwm
                elif dx < 0 and dy > 0:
                    pwm_a = sector_pwm
                    pwm_b = rest_pwm
                    pwm_c = rest_pwm
                    pwm_d = rest_pwm
                elif dx < 0 and dy < 0:
                    pwm_b = sector_pwm
                    pwm_a = rest_pwm
                    pwm_c = rest_pwm
                    pwm_d = rest_pwm
                elif dx > 0 and dy < 0:
                    pwm_c = sector_pwm
                    pwm_a = rest_pwm
                    pwm_b = rest_pwm
                    pwm_d = rest_pwm
                elif dx > 0 and dy > 0:
                    pwm_d = sector_pwm
                    pwm_a = rest_pwm
                    pwm_b = rest_pwm
                    pwm_c = rest_pwm
                elif dx < 0:
                    pwm_a = pwm_b = sector_pwm
                    pwm_c = pwm_d = rest_pwm
                elif dx > 0:
                    pwm_c = pwm_d = sector_pwm
                    pwm_a = pwm_b = rest_pwm
                elif dy < 0:
                    pwm_b = pwm_c = sector_pwm
                    pwm_a = pwm_d = rest_pwm
                elif dy > 0:
                    pwm_a = pwm_d = sector_pwm
                    pwm_b = pwm_c = rest_pwm
