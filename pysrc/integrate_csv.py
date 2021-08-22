import cv2
import os
import csv
import time
import argparse

if __name__ == '__main__':

    parser = argparse.ArgumentParser("./integrate_csv.py")

    parser.add_argument(
        '--file_counter', '-c',
        type=int,
        required=True,
        default=1,
        help='いくつのファイルを結合するか決定する関数',
    )
    parser.add_argument(
        '--file_root_path', '-p',
        type=str,
        required=False,
        default='/home/amsl/kawai_airsim/2021_05/',
        help='Saved data top path',
    )
    parser.add_argument(
        '--integrated_file_name', '-n',
        type=str,
        required=False,
        default='random_place.csv',
    )
    parser.add_argument(
        '--saved_csv_name', '-s',
        type=str,
        required=False,
        default='pose_camera',
    )
    
    FLAGS, unparsed = parser.parse_known_args()

    csv_master_data = []
    
    for i in range(FLAGS.file_counter):
        csv_counter = str(i)
        csv_path = FLAGS.file_root_path + FLAGS.saved_csv_name + csv_counter + '.csv'

        try:
            with open(csv_path) as f:
                reader = csv.reader(f)
                for row in reader:
                    csv_master_data.append(row)
                
                print('CSV file %s loaded!', csv_path)
        except Exception as e:
            print(e)
            print("Error opening csv file %s!! Exit...", csv_path)
            quit()

    print("Load CSV file done. Save to integrated file")

    integrated_csv_path = FLAGS.file_root_path + FLAGS.integrated_file_name

    with open(integrated_csv_path, 'w') as w:
        writer = csv.writer(w)
        for row in csv_master_data:
            writer.writerow(row)

    print("Save integrated file %s!! Exit...", integrated_csv_path)

