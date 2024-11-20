import React, { useState } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polyline } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

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

type Mode = 'node' | 'edge';

interface NodeInfo {
  nodeId: string;
  position: [number, number];
}

interface EdgeInfo {
  fromNodeId: string;
  toNodeId: string;
  fromPosition: [number, number];
  toPosition: [number, number];
}

interface MapProps {
  center: [number, number];
  zoom: number;
  mode: Mode;
  permanentNodes: NodeInfo[];
  onNodeSelect: (nodeData: NodeInfo | null) => void;
  onEdgeSelect: (edgeData: EdgeInfo | null) => void;
  onPermanentNodeClick: (nodeId: string) => void;
}

const SelectionHandler: React.FC<{ 
  mode: Mode;
  permanentNodes: NodeInfo[];
  onNodeSelect: (data: NodeInfo | null) => void;
  onEdgeSelect: (data: EdgeInfo | null) => void;
  onPermanentNodeClick: (nodeId: string) => void;
}> = ({ mode, permanentNodes, onNodeSelect, onEdgeSelect, onPermanentNodeClick }) => {
  const map = useMapEvents({
    click: async (e) => {
      const { lat, lng } = e.latlng;
      
      // Check if clicking on a permanent node first
      const clickRadius = 20; // pixels
      const clickPoint = map.latLngToContainerPoint(e.latlng);
      const clickedPermanentNode = permanentNodes.find(node => {
        const markerPoint = map.latLngToContainerPoint(L.latLng(node.position[0], node.position[1]));
        const distance = clickPoint.distanceTo(markerPoint);
        return distance < clickRadius;
      });

      if (clickedPermanentNode) {
        onPermanentNodeClick(clickedPermanentNode.nodeId);
        return;
      }
      
      try {
        if (mode === 'node') {
          const query = `
            [out:json];
            node(around:20,${lat},${lng});
            out body;
          `;
          
          const response = await fetch('https://overpass-api.de/api/interpreter', {
            method: 'POST',
            body: query,
          });
          
          const data = await response.json();
          
          if (data.elements.length > 0) {
            const nearestNode = data.elements[0];
            onNodeSelect({
              nodeId: nearestNode.id.toString(),
              position: [nearestNode.lat, nearestNode.lon]
            });
          } else {
            onNodeSelect(null);
          }
        } else {
          const query = `
            [out:json];
            way(around:20,${lat},${lng})[highway];
            (._;>;);
            out body;
          `;
          
          const response = await fetch('https://overpass-api.de/api/interpreter', {
            method: 'POST',
            body: query,
          });
          
          const data = await response.json();
          
          if (data.elements.length > 0) {
            const ways = data.elements.filter((el: any) => el.type === 'way');
            if (ways.length > 0) {
              const nearestWay = ways[0];
              const fromNodeId = nearestWay.nodes[0].toString();
              const toNodeId = nearestWay.nodes[nearestWay.nodes.length - 1].toString();
              
              const nodes = data.elements.filter((el: any) => el.type === 'node');
              const fromNode = nodes.find((n: any) => n.id.toString() === fromNodeId);
              const toNode = nodes.find((n: any) => n.id.toString() === toNodeId);
              
              if (fromNode && toNode) {
                onEdgeSelect({
                  fromNodeId,
                  toNodeId,
                  fromPosition: [fromNode.lat, fromNode.lon],
                  toPosition: [toNode.lat, toNode.lon]
                });
              }
            }
          } else {
            onEdgeSelect(null);
          }
        }
      } catch (error) {
        console.error('Error fetching data:', error);
        mode === 'node' ? onNodeSelect(null) : onEdgeSelect(null);
      }
    },
  });
  
  return null;
};

const MapComponent: React.FC<MapProps> = ({ 
  center, 
  zoom, 
  mode,
  permanentNodes,
  onNodeSelect,
  onEdgeSelect,
  onPermanentNodeClick
}) => {
  const [selectedNode, setSelectedNode] = useState<NodeInfo | null>(null);
  const [selectedEdge, setSelectedEdge] = useState<EdgeInfo | null>(null);

  const handleNodeSelect = (node: NodeInfo | null) => {
    setSelectedNode(node);
    setSelectedEdge(null);
    onNodeSelect(node);
  };

  const handleEdgeSelect = (edge: EdgeInfo | null) => {
    setSelectedEdge(edge);
    setSelectedNode(null);
    onEdgeSelect(edge);
  };

  return (
    <div style={{ height: '400px', width: '100%' }}>
      <MapContainer 
        center={center} 
        zoom={zoom} 
        style={{ height: '100%', width: '100%' }}
      >
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        <SelectionHandler 
          mode={mode} 
          permanentNodes={permanentNodes}
          onNodeSelect={handleNodeSelect} 
          onEdgeSelect={handleEdgeSelect}
          onPermanentNodeClick={onPermanentNodeClick}
        />
        
        {/* Permanent Nodes */}
        {permanentNodes.map((node) => (
          <Marker 
            key={node.nodeId} 
            position={node.position}
            icon={blackIcon}
          >
            <Popup>
              Node ID: {node.nodeId}
              <br />
              Click to remove
            </Popup>
          </Marker>
        ))}
        
        {/* Temporary Selection Display */}
        {selectedNode && mode === 'node' && (
          <Marker position={selectedNode.position}>
            <Popup>Node ID: {selectedNode.nodeId}</Popup>
          </Marker>
        )}
        
        {selectedEdge && mode === 'edge' && (
          <>
            <Marker position={selectedEdge.fromPosition}>
              <Popup>Start Node: {selectedEdge.fromNodeId}</Popup>
            </Marker>
            <Marker position={selectedEdge.toPosition}>
              <Popup>End Node: {selectedEdge.toNodeId}</Popup>
            </Marker>
            <Polyline
              positions={[selectedEdge.fromPosition, selectedEdge.toPosition]}
              color="blue"
              weight={3}
              opacity={0.7}
            />
          </>
        )}
      </MapContainer>
    </div>
  );
};

export default MapComponent;
