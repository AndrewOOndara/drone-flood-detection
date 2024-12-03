import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Rectangle, Polyline } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import L from 'leaflet';

// Fix for default marker icons
delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png'),
});

// Custom black icon for permanent nodes
const blackIcon = new L.Icon({
  iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-2x-black.png',
  shadowUrl: 'https://cdnjs.cloudflare.com/ajax/libs/leaflet/0.7.7/images/marker-shadow.png',
  iconSize: [25, 41],
  iconAnchor: [12, 41],
  popupAnchor: [1, -34],
  shadowSize: [41, 41]
});

// Types
interface NodeInfo {
  nodeId: string;
  position: [number, number];
}

interface PathSegment {
  fromNode: NodeInfo;
  toNode: NodeInfo;
}

interface Rectangle {
  id: string;
  bounds: [number, number, number, number]; // [x0, x1, y0, y1]
  name: string;
}

interface MapProps {
  center: [number, number];
  zoom: number;
  permanentNodes: NodeInfo[];
  onNodeSelect: (nodeData: NodeInfo | null) => void;
  onPermanentNodeClick: (nodeId: string) => void;
  plannedPath: PathSegment[] | null;
}


// Helper function to check if a point is inside a rectangle
const isPointInRectangle = (lat: number, lng: number, rect: [number, number, number, number]): boolean => {
  const [x0, x1, y0, y1] = rect;
  return lng >= x0 && lng <= x1 && lat >= y0 && lat <= y1;
};

// Hoverable Rectangle Component
const HoverableRectangle: React.FC<{
  bounds: L.LatLngBoundsExpression;
  name: string;
}> = ({ bounds, name }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <Rectangle
      bounds={bounds}
      pathOptions={{ 
        color: isHovered ? '#4a90e2' : 'blue',
        weight: isHovered ? 3 : 1,
        fillOpacity: isHovered ? 0.3 : 0.2,
        fillColor: 'blue',
        className: isHovered ? 'rectangle-hover' : ''
      }}
      eventHandlers={{
        mouseover: () => setIsHovered(true),
        mouseout: () => setIsHovered(false)
      }}
    >
      <Popup>{name}</Popup>
    </Rectangle>
  );
};

// Selection Handler Component
const SelectionHandler: React.FC<{ 
  rectangles: Rectangle[];
  permanentNodes: NodeInfo[];
  onNodeSelect: (data: NodeInfo | null) => void;
  onPermanentNodeClick: (nodeId: string) => void;
}> = ({ rectangles, permanentNodes, onNodeSelect, onPermanentNodeClick }) => {
  const map = useMapEvents({
    click: async (e) => {
      const { lat, lng } = e.latlng;
      
      // Check if click is within any restricted rectangle
      const isInRestrictedArea = rectangles.some(rect => 
        isPointInRectangle(lat, lng, rect.bounds)
      );

      if (isInRestrictedArea) {
        return;
      }

      // Check if clicking on a permanent node
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
          // Double check that the node isn't in a restricted area
          if (!rectangles.some(rect => 
            isPointInRectangle(nearestNode.lat, nearestNode.lon, rect.bounds)
          )) {
            onNodeSelect({
              nodeId: nearestNode.id.toString(),
              position: [nearestNode.lat, nearestNode.lon]
            });
          }
        } else {
          onNodeSelect(null);
        }
      } catch (error) {
        console.error('Error fetching data:', error);
        onNodeSelect(null);
      }
    },
  });
  
  return null;
};

// Main Map Component

// Mock API with shifting rectangles
const fetchRectangles = async (): Promise<Rectangle[]> => {
  await new Promise(resolve => setTimeout(resolve, 800));
  
  // Get a small random shift for the rectangles
  const shift = (Math.random() - 0.5) * 0.02; // Random shift between -0.01 and 0.01

  return [
    {
      id: 'rect-1',
      bounds: [-0.09 + shift, -0.08 + shift, 51.5, 51.51],
      name: 'Restricted Zone A'
    },
    {
      id: 'rect-2',
      bounds: [-0.11 + shift, -0.09 + shift, 51.49, 51.505],
      name: 'Restricted Zone B'
    },
    {
      id: 'rect-3',
      bounds: [-0.13 + shift, -0.12 + shift, 51.51, 51.52],
      name: 'Restricted Zone C'
    }
  ];
};

// Previous helper functions and sub-components remain the same...

const UserMap: React.FC<MapProps> = ({ 
  center, 
  zoom, 
  permanentNodes,
  onNodeSelect,
  onPermanentNodeClick,
  plannedPath
}) => {
  const [rectangles, setRectangles] = useState<Rectangle[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedNode, setSelectedNode] = useState<NodeInfo | null>(null);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [lastRefreshTime, setLastRefreshTime] = useState<Date>(new Date());

  const loadRectangles = async (isRefresh: boolean = false) => {
    if (isRefresh) {
      setIsRefreshing(true);
    } else {
      setIsLoading(true);
    }
    setError(null);

    try {
      const fetchedRectangles = await fetchRectangles();
      setRectangles(fetchedRectangles);
      setLastRefreshTime(new Date());
    } catch (err) {
      setError('Failed to load restricted areas');
      console.error('Error fetching rectangles:', err);
    } finally {
      setIsLoading(false);
      setIsRefreshing(false);
    }
  };

  // Initial load
  useEffect(() => {
    loadRectangles();
  }, []);

  // Refresh every 10 seconds
  useEffect(() => {
    const intervalId = setInterval(() => {
      loadRectangles(true);
    }, 10000);

    return () => clearInterval(intervalId);
  }, []);

  const handleNodeSelect = (node: NodeInfo | null) => {
    setSelectedNode(node);
    onNodeSelect(node);
  };

  const handleManualRefresh = () => {
    loadRectangles(true);
  };

  if (isLoading) {
    return <div className="flex items-center justify-center h-[400px] bg-gray-100">
      Loading restricted areas...
    </div>;
  }

  if (error) {
    return (
      <div className="h-[400px] bg-red-50 p-4">
        <div className="text-red-600 mb-4">{error}</div>
        <button
          onClick={handleManualRefresh}
          className="px-4 py-2 bg-red-600 text-white rounded hover:bg-red-700"
        >
          Try Again
        </button>
      </div>
    );
  }

  return (
    <>
      <style>
        {`
          .rectangle-hover {
            filter: drop-shadow(0 0 10px rgba(74, 144, 226, 0.7));
            transition: all 0.3s ease;
          }
          .planned-path {
            animation: dashdraw 15s linear infinite;
          }
          @keyframes dashdraw {
            to {
              stroke-dashoffset: -1000;
            }
          }
          .refresh-spinner {
            animation: spin 1s linear infinite;
          }
          @keyframes spin {
            to {
              transform: rotate(360deg);
            }
          }
        `}
      </style>
      
      {/* Refresh status bar */}
      <div className="mb-2 flex justify-between items-center bg-gray-100 p-2 rounded">
        <div className="text-sm text-gray-600">
          Last updated: {lastRefreshTime.toLocaleTimeString()}
        </div>
        <button
          onClick={handleManualRefresh}
          disabled={isRefreshing}
          className={`flex items-center px-3 py-1 rounded ${
            isRefreshing 
              ? 'bg-gray-300 cursor-not-allowed' 
              : 'bg-blue-500 hover:bg-blue-600 text-white'
          }`}
        >
          {isRefreshing ? (
            <>
              <svg 
                className="animate-spin -ml-1 mr-2 h-4 w-4 text-white" 
                xmlns="http://www.w3.org/2000/svg" 
                fill="none" 
                viewBox="0 0 24 24"
              >
                <circle 
                  className="opacity-25" 
                  cx="12" 
                  cy="12" 
                  r="10" 
                  stroke="currentColor" 
                  strokeWidth="4"
                />
                <path 
                  className="opacity-75" 
                  fill="currentColor" 
                  d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"
                />
              </svg>
              Refreshing...
            </>
          ) : (
            <>
              <svg 
                className="w-4 h-4 mr-1" 
                fill="none" 
                stroke="currentColor" 
                viewBox="0 0 24 24"
              >
                <path 
                  strokeLinecap="round" 
                  strokeLinejoin="round" 
                  strokeWidth={2} 
                  d="M4 4v5h.582m15.356 2A8.001 8.001 0 004.582 9m0 0H9m11 11v-5h-.581m0 0a8.003 8.003 0 01-15.357-2m15.357 2H15" 
                />
              </svg>
              Refresh
            </>
          )}
        </button>
      </div>

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
          
          {/* Rectangles with transition effect */}
          {rectangles.map((rect) => (
            <HoverableRectangle
              key={rect.id}
              bounds={[[rect.bounds[2], rect.bounds[0]], [rect.bounds[3], rect.bounds[1]]]}
              name={`${rect.name} (Updated: ${lastRefreshTime.toLocaleTimeString()})`}
            />
          ))}

          {/* Rest of the component remains the same... */}
          {plannedPath && plannedPath.map((segment, index) => (
            <Polyline
              key={index}
              positions={[
                segment.fromNode.position,
                segment.toNode.position
              ]}
              pathOptions={{
                color: '#22c55e',
                weight: 4,
                opacity: 0.8,
                dashArray: '10, 10',
                className: 'planned-path'
              }}
            >
              <Popup>
                Edge {index + 1}: {segment.fromNode.nodeId} → {segment.toNode.nodeId}
              </Popup>
            </Polyline>
          ))}

          <SelectionHandler 
            rectangles={rectangles}
            permanentNodes={permanentNodes}
            onNodeSelect={handleNodeSelect} 
            onPermanentNodeClick={onPermanentNodeClick}
          />
          
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
          
          {selectedNode && (
            <Marker position={selectedNode.position}>
              <Popup>Node ID: {selectedNode.nodeId}</Popup>
            </Marker>
          )}
        </MapContainer>
      </div>
    </>
  );
};

export default UserMap;
