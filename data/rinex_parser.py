import georinex as gr
import numpy as np
import xarray as xr
import json

# rinex files with rover obs data
rover_data_file1 = '50BR_VRS1018n.24o'
rover_data_file2 = '50BR_VRS1018o.24o'

# rinex files with station obs data
station_data_file1 = 'int.50BRM018n.24o'
station_data_file2 = 'int.50BRM018o.24o'

# C1W, C2W - pseudo range; L1W, L2W - carrier phase
rover_obs1 = gr.load(rover_data_file1, use='G', meas=['C1W', 'L1W', 'C2W', 'L2W']).dropna(dim='time', how='all')
rover_obs2 = gr.load(rover_data_file2, use='G', meas=['C1W', 'L1W', 'C2W', 'L2W']).dropna(dim='time', how='all')
station_obs1 = gr.load(station_data_file1, use='G', meas=['C1W', 'L1W', 'C2W', 'L2W']).dropna(dim='time', how='all')
station_obs2 = gr.load(station_data_file2, use='G', meas=['C1W', 'L1W', 'C2W', 'L2W']).dropna(dim='time', how='all')

rover_obs = xr.concat((rover_obs1, rover_obs2), dim='time')
station_obs = xr.concat((station_obs1, station_obs2), dim='time')

times_rover_obs = rover_obs.time.data
times_station_obs = station_obs.time.data


rover_dct_to_save = {}
for t in times_rover_obs:
    obs_rover_pd_temp = rover_obs.sel(time=t).to_pandas().reset_index().dropna()
    del obs_rover_pd_temp['time']
    obs_rover_pd_temp = {str(t): obs_rover_pd_temp.to_dict('list')}
    rover_dct_to_save.update(obs_rover_pd_temp)

station_dct_to_save = {}
for t in times_station_obs:
    obs_station_pd_temp = station_obs.sel(time=t).to_pandas().reset_index().dropna()
    del obs_station_pd_temp['time']
    obs_station_pd_temp = {str(t): obs_station_pd_temp.to_dict('list')}
    station_dct_to_save.update(obs_station_pd_temp)


with open('rover_data.json', 'w') as f:
    json.dump(rover_dct_to_save, f)

with open('station_data.json', 'w') as f:
    json.dump(station_dct_to_save, f)

np.savetxt("rover_times.csv", times_rover_obs, delimiter=',')


