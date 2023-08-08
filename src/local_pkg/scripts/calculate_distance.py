import csv
import math

def calculate_distance(x_values, y_values):
    distance = 0.0
    for i in range(1, len(x_values)):
        x1, y1 = x_values[i-1],y_values[i-1]
        x2, y2 = x_values[i],y_values[i]
        segment_distance = math.sqrt((x2-x1)**2 + (y2-y1)**2)
        distance += segment_distance
    return distance

def read_csv_file(file_path):
    x_values = []
    y_values = []
    with open(file_path, 'r') as csv_file :
        reader = csv.reader(csv_file)
        next(reader)
        for row in reader:
            x_values.append(float(row[0]))
            y_values.append(float(row[1]))
    return x_values, y_values

def main():
    file_path = './encoder_2.csv'
    x_values , y_values = read_csv_file(file_path)
    distance = calculate_distance(x_values, y_values)
    print('total distance : ', distance)

if __name__ == "__main__":
    main()

