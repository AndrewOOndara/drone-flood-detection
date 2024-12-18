import React, { useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polyline, useMap, CircleMarker } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';
import { EdgeInfo, LatLon, NodeInfo } from '../types';

// Fix for default marker icons
delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png'),
});

// Create a black icon for permanent nodes
const blackIcon = new L.Icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-black.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41]
});



type MapProps = {
  center: [number, number],
  radius: number,
  // onNodeSelect: null | ((coord: LatLon) => void),
  // onEdgeSelect: null | ((edgeData: EdgeInfo) => void),
  // displayNodes: {[k: string]: NodeInfo},
  // displayEdges: EdgeInfo[],
  children: React.ReactNode
  // onPermanentNodeClick: (nodeId: string) => void,
}



// const SelectionHandler = ({ onNodeSelect }: Pick<MapProps, "onNodeSelect">) => {
//   const [justClicked, setJustClicked] = React.useState<{ [k: string]: [number, number] }>({});
//   const clickCb = React.useCallback(async (e: L.LeafletMouseEvent) => {
//     const { lat, lng } = e.latlng;

//     // https://stackoverflow.com/a/8084248
//     const clickId = (Math.random() + 1).toString(36).substring(3);

//     setJustClicked(clicks => ({
//       ...clicks,
//       [clickId]: [lat, lng]
//     }));
//     try {
//       onNodeSelect?.([lat, lng]);
//       // else if (onEdgeSelect) {
//       //   const v = await queryForEdge(lat, lng);
//       //   if (v) onEdgeSelect(v);
//       // }
//     }
//     finally {
//       setJustClicked(clicks => {
//         let cloned = { ...clicks };
//         delete cloned[clickId];
//         return cloned;
//       });
//     }

//     // // Check if clicking on a permanent node first
//     // const clickRadius = 20; // pixels
//     // const clickPoint = map.latLngToContainerPoint(e.latlng);
//     // const clickedPermanentNode = permanentNodes.find(node => {
//     //   const markerPoint = map.latLngToContainerPoint(L.latLng(node.position[0], node.position[1]));
//     //   const distance = clickPoint.distanceTo(markerPoint);
//     //   return distance < clickRadius;
//     // });

//     // if (clickedPermanentNode) {
//     //   onPermanentNodeClick(clickedPermanentNode.nodeId);
//     //   return;
//     // }
//   }, [setJustClicked, onNodeSelect]);
//   const map = useMapEvents({
//     click: clickCb,
//   });

//   return <>
//       {Object.entries(justClicked).map(([key, pos]) => {
//         <CircleMarker
//           key={key}
//           center={pos}
//           radius={12}
//           color="orange"
//         />
//       })}
//     </>
// };

const MapComponent = React.memo(function ({
  center,
  radius,
  // onNodeSelect,
  // displayNodes,
  // displayEdges,
  children
}: MapProps) {
  return (
    <MapContainer
      center={[0.0, 0.0]}
      zoom={13}
      style={{ height: '100%', width: '100%' }}
    >
      <MapUpdater center={center} radius={radius} />
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />
      {children}
      {/* <SelectionHandler
        onNodeSelect={onNodeSelect}
      /> */}
      {/* Temporary Selection Display */}
      {/* {selectedNode && mode === 'node' && (
        <Marker position={selectedNode.position}>
          <Popup>Node ID: {selectedNode.nodeId}</Popup>
        </Marker>
      )} */}
          {/* <Popup>
            Node ID: {node.nodeId}
            <br />
            Click to remove
          </Popup> */}

      {/* {Object.entries(displayNodes).map(([k, node]) => (
        <Marker
          key={k}
          position={node.position}
          icon={blackIcon}
        >
        </Marker>
      ))}

      {displayEdges.map((edge) =>
        <EdgeLine
          key={`${edge.fromNodeId}:${edge.toNodeId}`}
          edge={edge}
        />
      )}
      <div style={{height: '400px', width: '400px', backgroundColor: 'white'}}>

      </div> */}
    </MapContainer>
  );
});

const EARTH_RADIUS_M = 6E6;
const MapUpdater = React.memo(function ({ center: [lat, lon], radius }: { center: [number, number], radius: number }) {
  const map = useMap();
  const lonAngle = radius * 180 / (EARTH_RADIUS_M * Math.PI);
  const latAngle = Math.cos(lon * Math.PI / 180) * radius * 180 / (EARTH_RADIUS_M * Math.PI);
  React.useEffect(() => {
    map.flyToBounds(L.latLngBounds(
      L.latLng(lat - latAngle, lon - lonAngle),
      L.latLng(lat + latAngle, lon + lonAngle),
    ), { animate: false })
  }, [lat, lon, map]);
  return null;
})

export default MapComponent;
