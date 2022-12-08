import numpy as np
import matplotlib.pyplot as plt

class DisplayOverlays:
    def __init__(self, writer, reader):
        self.w, self.r = writer, reader

    def get_display_coords(self, x, y, display=(360, 360), world=(30, 16)):
        x = (display[0]*0.5) - (x * (display[0]/world[0]))
        y = display[1] - ((display[1]*0.5) - (y * (display[1]/world[1])))
        x, y = np.clip(x, 0, display[0]-1), np.clip(y, 0, display[1]-1)
        return int(x), int(y)

    def update_depth_display(self, image, width):
        # Convert greyscale image to color
        image = image.reshape(-1, width).T
        image = plt.imshow(image)
        image = image.cmap(image.norm(image.get_array()))
        image = np.delete(image, 3, 2)*255

        # Update depth display
        depth_display = self.r.device.depth_display
        ir = depth_display.imageNew(image.tolist(), depth_display.RGB)
        depth_display.imagePaste(ir, 0, 0, False)
        depth_display.imageDelete(ir)

    def draw_object_bounds(self, display, bounds, color=0xFF0000):
        display.setColor(color)
        for bound in bounds:
            (X1, Y1), (X2, Y2) = bound
            x_min, y_min = min(X1,X2), min(Y1,Y2)
            x_max, y_max = max(X1,X2), max(Y1,Y2)
            display.drawRectangle(min(X1,X2), min(Y1,Y2), abs(X2-X1)+1, abs(Y2-Y1)+1)

    def draw_robot_position(self):
        display = self.w.device.display
        display.setColor(0xFF0000)
        pose = self.r.robot.pose
        rx, ry = self.get_display_coords(pose.x, pose.y)
        display.drawPixel(rx, ry)

    def display_point_cloud(self, point_cloud):
        map = self.r.env.map
        display = self.w.device.display

        if (point_cloud is None): return
        for x, y in point_cloud:
            x, y = self.get_display_coords(x, y)
            pixel = np.clip(map.pixel(x, y) + 5e-3, 0.0, 1.0)
            map.update_pixel(x, y, pixel)

            g = int(pixel*255)
            color = int(g*256**2 + g*256 + g)
            display.setColor(color)
            display.drawPixel(x, y)

        self.w.env.map = map

    def draw_estimated_location_on_map(self):
        display = self.r.device.display
        object_location = np.array(self.r.env.object_location)

        display.setColor(0xFFFF00)
        for x,y,_ in object_location:
            x, y = self.get_display_coords(x, y)
            display.drawRectangle(x-5, y-5, 10,10)
    
    def redraw_display(self, map):
        width, height = map.shape
        map = map.T*255
        map = np.dstack([map, map, map]).tolist()
        # Reload new map
        display = self.r.device.display
        ir = display.imageNew(map, display.RGB, width, height)
        display.imagePaste(ir, 0, 0, False)
        display.imageDelete(ir)

    def draw_waypoints(self):
        # Draw waypoints on display as green path
        display = self.r.device.display
        display.setColor(0x00FF00)
        for waypoint in self.r.env.waypoints:
            wx, wy = self.get_display_coords(waypoint[0],waypoint[1])
            display.drawPixel(wx, wy)