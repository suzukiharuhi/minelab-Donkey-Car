class Controller:
    def __init__(self):
        pass
    
    def angle_diff(self, current, reference):
        if abs(current - reference) < 2:
            return 0
        diff = (current - reference + 360) % 360
        return diff
    
    def adjust_angle(self, x_distance, z_distance):
        if z_distance > 150:
            threshold = 20
        else:
            threshold = 10
        if x_distance < (-1 * threshold):
            self.publish_message("slow_turn_left")
        elif x_distance > threshold:
            self.publish_message("slow_turn_right")
        else: # -〇 < x < 〇
            self.publish_message("move_forward")   