import yaml

with open('random_passage_data.yaml','r') as file:
  data = yaml.safe_load(file)

with open('laas_passage_data.yaml','r') as file:
  data2 = yaml.safe_load(file)

with open('bremen_passage_data.yaml','r') as file:
  data3 = yaml.safe_load(file)

acc = sum(data['t_detect']+data2['t_detect']+data3['t_detect'])/200 #len(data['detections'])
#acc = acc + sum(data['f_detect']+data2['f_detect']+data3['f_detect'])/200 #len(data['detections'])
#acc = acc + sum(data['no_detect_lim']+data2['no_detect_lim']+data3['no_detect_lim'])/200 #len(data['detections'])
#acc = acc + sum(data['no_detect']+data2['no_detect']+data3['no_detect'])/200 #len(data['detections'])
#acc = sum(data['detections'])/100. #len(data['detections'])
print(acc)
