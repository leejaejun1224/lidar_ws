import json
import numpy as np
import matplotlib.pyplot as plt
import cubic_spline_planner as cs
class making_map():
    def __init__(self):
        self.dir = "/home/gigacha/TEAM-GIGACHA/src/semi_pkg/scripts/maps/Inha_Songdo/"
        self.map_name = "songdo_backward_driving_field_test_AJJ.json"
        self.maptxt = "tmp.txt"
        print("-----------------------------")
        print("Making map module initiated!")
        print("-----------------------------")
        self.data = {}


    def main(self):
        # with open(self.dir+self.map_name, 'r') as outfile:
        #     data=outfile.readlines()
        # print(data)
        print("-------")
        x_in = [4.61, 4.91, 5.25, 5.56, 5.89, 6.29, 6.75, 7.1, 7.55, 7.84, 8.1, 8.33, 8.5, 8.63, 8.68, 8.67, 8.58, 8.44, 8.27, 8.19, 8.19, 8.32, 8.65, 8.92, 9.27, 9.86, 10.56, 11.15, 11.81, 12.5, 13.21, 13.92, 14.64, 15.13, 15.6, 16.3, 16.72, 17.12, 17.51, 17.86, 18.31, 18.71, 19.06, 19.4, 19.73, 20.1, 20.48, 20.86, 21.22, 21.6, 21.99, 22.44, 22.88, 23.37, 23.83, 24.3, 24.66, 25.0, 25.34, 25.69, 26.08, 26.46, 27.09]
        y_in = [-0.04, 0.39, 0.82, 1.23, 1.61, 2.03, 2.52, 2.89, 3.43, 3.84, 4.28, 4.78, 5.28, 5.83, 6.43, 6.97, 7.71, 8.21, 8.82, 9.36, 10.03, 10.8, 11.62, 12.15, 12.64, 13.24, 13.75, 14.05, 14.28, 14.45, 14.55, 14.65, 14.79, 14.94, 15.13, 15.47, 15.76, 16.09, 16.47, 16.87, 17.47, 18.03, 18.57, 19.03, 19.43, 19.88, 20.3, 20.71, 21.06, 21.43, 21.79, 22.15, 22.46, 22.78, 23.16, 23.61, 23.96, 24.36, 24.76, 25.2, 25.57, 25.95, 26.5]
        print(x_in)
        print(y_in)
        # x1, y1 = map(float, input("Start point : ").split())
        # x2, y2 = map(float, input("End point : ").split())
        # x_in = [x1, x2]
        # y_in = [y1, y2]
        x, y = cs.main(x_in, y_in)
        
        for i in range(len(x)):
            self.data[i] = [x[i], y[i], "parking", 5]
        
        with open(self.dir+self.map_name, 'w') as outfile:
            json.dump(self.data, outfile, indent=4)

        print("\nfinished!", end="\n\n")

if __name__ == "__main__":
    mm = making_map()
    mm.main()


