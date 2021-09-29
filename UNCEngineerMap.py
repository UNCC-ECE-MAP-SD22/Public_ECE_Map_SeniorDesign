#import gmplot package
#https://www.geeksforgeeks.org/python-plotting-google-map-using-gmplot-package/
import gmplot as gmplot
  
latitude_list = [ 35.309838, 35.311365]
longitude_list = [ -80.741831, -80.741292]
  
gmap3 = gmplot.GoogleMapPlotter(35.309297306493505,
                                -80.74090250710788, 13)
  
# scatter method of map object 
# scatter points on the google map
gmap3.scatter( latitude_list, longitude_list, 'blue',
                              size = 15, marker = 'o' )
  
# Plot method Draw a line in
# between given coordinates
gmap3.plot(latitude_list, longitude_list, 
           'red', edge_width = 2.0)
  
gmap3.draw( "C:\\Users\\micha\\Senior Design\\map13.html" )