# beepbop
A tiny routing engine


## Build

Install mason to manage dependencies

```
git clone --depth 1 --branch v0.14.1 https://github.com/mapbox/mason.git .mason
```

Then build with c`make`:
```
mkdir build
cd build
cmake ..
make
```

## Run

Download an OSM extract.

```
wget http://download.geofabrik.de/north-america/us/district-of-columbia-latest.osm.pbf
```

Build the graph.

```
./build_graph OSM_FILE
```
