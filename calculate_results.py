import xml.etree.ElementTree as ET

# 解析 tripinfo.xml
tree = ET.parse('SUMO\\test1-tripinfo.xml')
root = tree.getroot()

# 初始化统计变量
total_delay = 0
num_vehicles = 0

# 遍历每辆车的数据
for trip in root.findall('tripinfo'):
    depart = float(trip.get('depart'))
    arrival = float(trip.get('arrival'))
    duration = float(trip.get('duration'))
    time_loss = float(trip.get('timeLoss'))  # 延误时间

    total_delay += time_loss
    num_vehicles += 1

# 计算平均延误
if num_vehicles > 0:
    average_delay = total_delay / num_vehicles
    print(f"Average delay: {average_delay} seconds")
else:
    print("No vehicles found in tripinfo.xml.")





# 解析 tripinfo.xml
tree = ET.parse('SUMO\\test2-tripinfo.xml')
root = tree.getroot()

# 初始化统计变量
total_delay = 0
num_vehicles = 0

# 遍历每辆车的数据
for trip in root.findall('tripinfo'):
    depart = float(trip.get('depart'))
    arrival = float(trip.get('arrival'))
    duration = float(trip.get('duration'))
    time_loss = float(trip.get('timeLoss'))  # 延误时间

    total_delay += time_loss
    num_vehicles += 1

# 计算平均延误
if num_vehicles > 0:
    average_delay = total_delay / num_vehicles
    print(f"Average delay: {average_delay} seconds")
else:
    print("No vehicles found in tripinfo.xml.")





# 解析 emission_output.xml
tree = ET.parse('SUMO\\test1-emission.xml')
root = tree.getroot()

# 初始化统计变量
total_co2 = 0
total_co = 0
total_nox = 0
total_hc = 0
total_pmx = 0
total_fuel = 0
num_vehicles = 0

# 遍历每辆车的排放数据
for timestep in root.findall('timestep'):
    for vehicle in timestep.findall('vehicle'):
        co2 = float(vehicle.get('CO2'))
        co = float(vehicle.get('CO'))
        nox = float(vehicle.get('NOx'))
        hc = float(vehicle.get('HC'))
        pmx = float(vehicle.get('PMx'))
        fuel = float(vehicle.get('fuel'))

        total_co2 += co2
        total_co += co
        total_nox += nox
        total_hc += hc
        total_pmx += pmx
        total_fuel += fuel

        num_vehicles += 1

# 计算平均排放
if num_vehicles > 0:
    avg_co2 = total_co2 / num_vehicles
    avg_co = total_co / num_vehicles
    avg_nox = total_nox / num_vehicles
    avg_hc = total_hc / num_vehicles
    avg_pmx = total_pmx / num_vehicles
    avg_fuel = total_fuel / num_vehicles

    print(f"Average CO2: {avg_co2} g")
    print(f"Average CO: {avg_co} g")
    print(f"Average NOx: {avg_nox} g")
    print(f"Average HC: {avg_hc} g")
    print(f"Average PMx: {avg_pmx} g")
    print(f"Average fuel: {avg_fuel} g")
else:
    print("No vehicles found in emission_output.xml.")





# 解析 emission_output.xml
tree = ET.parse('SUMO\\test2-emission.xml')
root = tree.getroot()

# 初始化统计变量
total_co2 = 0
total_co = 0
total_nox = 0
total_hc = 0
total_pmx = 0
total_fuel = 0
num_vehicles = 0

# 遍历每辆车的排放数据
for timestep in root.findall('timestep'):
    for vehicle in timestep.findall('vehicle'):
        co2 = float(vehicle.get('CO2'))
        co = float(vehicle.get('CO'))
        nox = float(vehicle.get('NOx'))
        hc = float(vehicle.get('HC'))
        pmx = float(vehicle.get('PMx'))
        fuel = float(vehicle.get('fuel'))

        total_co2 += co2
        total_co += co
        total_nox += nox
        total_hc += hc
        total_pmx += pmx
        total_fuel += fuel
        num_vehicles += 1

# 计算平均排放
if num_vehicles > 0:
    avg_co2 = total_co2 / num_vehicles
    avg_co = total_co / num_vehicles
    avg_nox = total_nox / num_vehicles
    avg_hc = total_hc / num_vehicles
    avg_pmx = total_pmx / num_vehicles
    avg_fuel = total_fuel / num_vehicles

    print(f"Average CO2: {avg_co2} g")
    print(f"Average CO: {avg_co} g")
    print(f"Average NOx: {avg_nox} g")
    print(f"Average HC: {avg_hc} g")
    print(f"Average PMx: {avg_pmx} g")
    print(f"Average fuel: {avg_fuel} g")
else:
    print("No vehicles found in emission_output.xml.")