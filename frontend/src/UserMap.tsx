import React, { useState, useEffect } from 'react';
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Rectangle, Polyline, useMap} from 'react-leaflet';
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
  from: [number, number];
  to: [number, number];
}

interface FloodZone {
  bbox: [number, number, number, number];
  confidence: number;
  drone_id: number;
  flooded: boolean;
  time: number;
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
  const [long_min, lat_min, long_max, lat_max] = rect;
  return lng >= long_min && lng <= long_max && lat >= lat_min && lat <= lat_max;
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

const UserMap: React.FC<MapProps> = ({ 
  center, 
  zoom, 
  permanentNodes,
  onNodeSelect,
  onPermanentNodeClick,
  plannedPath
}) => {

  const initialBounds = new L.LatLngBounds(
    [center[0] - 0.1, center[1] - 0.1], // Southwest corner
    [center[0] + 0.1, center[1] + 0.1]  // Northeast corner
  );

  const [rectangles, setRectangles] = useState<Rectangle[]>([]);
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);
  const [selectedNode, setSelectedNode] = useState<NodeInfo | null>(null);
  const [isRefreshing, setIsRefreshing] = useState(false);
  const [lastRefreshTime, setLastRefreshTime] = useState<Date>(new Date());
  const [mapBounds, setMapBounds] = useState<L.LatLngBounds>(initialBounds);

  
  const BoundsTracker = () => {
    const map = useMapEvents({
      moveend: () => {
        setMapBounds(map.getBounds());
      },
      zoomend: () => {
        setMapBounds(map.getBounds());
      }
    });
    
    // Set initial bounds
    useEffect(() => {
      setMapBounds(map.getBounds());
    }, [map]);

    return null;
  };

  const loadRectangles = async (isRefresh: boolean = false) => {
    console.log("Loadin rectangles")
    console.log(mapBounds)
    if (!mapBounds) return;

    if (isRefresh) {
      setIsRefreshing(true);
    } else {
      setIsLoading(true);
    }
    setError(null);

    console.log("Refresh vibe")

    const params = new URLSearchParams({
      x0: mapBounds.getWest().toString(),
      x1: mapBounds.getEast().toString(),
      y0: mapBounds.getSouth().toString(),
      y1: mapBounds.getNorth().toString(),
    });

    console.log(params)

    try {
      const response = await fetch(`http://168.5.58.43:5000/api/v1/zones?${params}`);
      
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      console.log("Response")
      const floodData: FloodZone[] = await response.json();
    
      // Transform flood data into rectangles, only keeping flooded areas
      const fetchedZones: Rectangle[] = floodData
        .filter(zone => zone.flooded)
        .map((zone, index) => ({
          id: `flood-zone-${index}`,
          bounds: zone.bbox,
          name: `Flood Zone ${index + 1}`
        }));      console.log(fetchedZones)
      // Assuming the API returns an array of rectangles in the format:
      // [{ id: string, bounds: [number, number, number, number], name: string }]
      setRectangles(fetchedZones);
      setLastRefreshTime(new Date());
    } catch (err) {
      setError('Failed to load restricted areas');
      console.error('Error fetching rectangles:', err);
    } finally {
      setIsLoading(false);
      setIsRefreshing(false);
    }
  };


  // // Initial load
  // useEffect(() => {
  //   console.log("In initial load")
  //   if (mapBounds) {
  //     loadRectangles(true);
  //   }
  // }, [mapBounds]);

  // Refresh every 10 seconds
  useEffect(() => {
    const intervalId = setInterval(() => {
      loadRectangles(true);
    }, 10000);

    return () => clearInterval(intervalId);
  }, [mapBounds]);

  const handleNodeSelect = (node: NodeInfo | null) => {
    setSelectedNode(node);
    onNodeSelect(node);
  };

  const handleManualRefresh = () => {
    if (mapBounds) {
      loadRectangles(true);
    }
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
          <BoundsTracker />
          <TileLayer
            attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
            url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
          />
          
          {rectangles.map((rect) => (
            <HoverableRectangle
              key={rect.id}
              bounds={[
                [rect.bounds[1], rect.bounds[0]], // [lat_min, long_min]
                [rect.bounds[3], rect.bounds[2]]  // [lat_max, long_max]
              ]}
              name={`${rect.name} (Updated: ${lastRefreshTime.toLocaleTimeString()})`}
            />
          ))}

          {plannedPath && plannedPath.map((segment, index) => (
            <Polyline
              key={index}
              positions={[
                segment.from,
                segment.to
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
                Edge {index + 1}
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