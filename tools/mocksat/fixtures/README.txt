Capture fixtures from real servers:

curl -o gibs_baseline.jpg 'https://gibs.earthdata.nasa.gov/wms/epsg4326/best/wms.cgi?SERVICE=WMS&VERSION=1.1.1&REQUEST=GetMap&STYLES=&SRS=EPSG:4326&LAYERS=GOES-East_ABI_GeoColor&BBOX=-70.0,13.0,-56.0,22.0&WIDTH=320&HEIGHT=176&FORMAT=image/jpeg'

curl -o eumet_progressive.jpg 'https://view.eumetsat.int/geoserver/ows?service=WMS&version=1.1.1&request=GetMap&styles=&srs=EPSG:4326&layers=mtg_fd:rgb_geocolour&bbox=-13.5,47.0,13.3,56.0&width=320&height=176&format=image/jpeg'

# Create a 320x176 all-black JPEG for validator tests
python3 -c "from PIL import Image; Image.new('RGB',(320,176)).save('black.jpg')"
