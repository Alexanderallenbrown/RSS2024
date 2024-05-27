import os
import pandas as pd
import sqlite3 as sql
import matplotlib.pyplot as plt

# Set the base directory
base_dir = r"C:.\Downloads\RSS2024-main"

# Set the directory where the files are located
directory = os.path.join(base_dir, "scripts", "casestudy_data")

# List the files you want to exclude
excluded_files = ['edge_slant', 'road_position_index', 'road_positions']

# Create a connection to the SQLite database
conn = sql.connect('case_study_data.db')

# Loop through all files in the directory
for filename in os.listdir(directory):
    if filename.endswith(".txt") and filename not in excluded_files:
        file_path = os.path.join(directory, filename)
        
        # Specify the column names explicitly as a list of strings
        column_names = ['time', 'goalRoll', 'Torque', 'speed', 'roll', 'rollrate', 'steer', 'steerrate', 'intE', 'goalLane', 'yaw', 'y']
        df = pd.read_csv(file_path, names=column_names)
        
        table_name = filename[:-4]
        
        # Create a table for the current dataframe
        df.to_sql(table_name, conn, if_exists='replace', index=False)
        
        print(f"Created table '{table_name}' in the SQLite database.")

# Close the connection to the SQLite database and check if data transmitted
conn.close()
print(df)
