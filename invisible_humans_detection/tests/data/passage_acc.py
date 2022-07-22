import yaml

with open('random_passage_data.yaml','r') as file:
  data = yaml.safe_load(file)

with open('laas_passage_data.yaml','r') as file:
  data2 = yaml.safe_load(file)

with open('bremen_passage_data.yaml','r') as file:
  data3 = yaml.safe_load(file)

acc = sum(data['detections']+data2['detections']+data3['detections'])/200 #len(data['detections'])
print(acc)
