#parse GPX data
route = open("craver_road.gpx", 'r')
route = route.read()

center_x = 0.0
center_y = 0.0

road_path = []

for i in range(len(route)):
    if i >= (len(route) - 6):
        break
    
    if (route[i] + route[i+1] + route[i+2] + route[i+3] + route[i+4] + route[i+5]) == "center":
        j = i+7
        lat = ""
        while route[j] != '%':
            lat += route[j]
            j += 1
        
        j += 3
        lon = ""
        while route[j] != '&':
            lon += route[j]
            j += 1
        print(lat, lon)
        
    if (route[i] + route[i+1] + route[i+2]) == "lat":
        j = i+5
        lat = ""
        while route[j] != '"':
            lat += route[j]
            j += 1
        
    
    if (route[i] + route[i+1] + route[i+2]) == "lon":
        j = i+5
        lon = ""
        while route[j] != '"':
            lon += route[j]
            j += 1
    