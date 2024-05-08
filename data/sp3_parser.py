import georinex as gr
import json


# Указываем путь к файлу SP3
sp3_file_path = 'GSR_1-3D_24011812.sp3'

# Загружаем данные из файла SP3
obs = gr.load(sp3_file_path, use='G').dropna(dim='time', how='all')

times_obs = obs.time.data

dct_to_save = {}
for t in times_obs:
    obs_pd_temp = obs.sel(time=t).to_dataframe().reset_index().dropna()
    obs_pd_temp = obs_pd_temp.groupby('sv').agg(list).reset_index()
    del obs_pd_temp['time']
    del obs_pd_temp['ECEF']
    obs_pd_temp['clock'] = obs_pd_temp['clock'].apply(lambda x: x[0])
    obs_pd_temp['dclock'] = obs_pd_temp['dclock'].apply(lambda x: x[0])
    obs_pd_temp = {str(t): obs_pd_temp.to_dict('list')}
    dct_to_save.update(obs_pd_temp)

with open('satellite_data.json', 'w') as f:
    json.dump(dct_to_save, f)


