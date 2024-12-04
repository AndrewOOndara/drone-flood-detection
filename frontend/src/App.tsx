import React, { useState } from 'react';
import UserMap from './UserMap';

interface NodeInfo {
  nodeId: string;
  position: [number, number];
}

interface PathSegment {
  from: [number, number];
  to: [number, number];
}

const planPath = async (coordinates: [number, number][]): Promise<PathSegment[]> => {
    try {
      const response = await fetch('http://168.5.58.43:5000/api/v1/find_path', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(coordinates)
      });
  
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
  
      const data = await response.json();
      console.log("data")
      console.log(data)
      console.log(data.length)
      // Convert the response into PathSegments
      const segments: PathSegment[] = [];
      for (let i = 0; i < data.length - 1; i++) {
        segments.push({
          from: data[i],
          to: data[i + 1]
        });
      }
      console.log("Size of segs")
      console.log(segments.length)
      return segments;
    } catch (error) {
      console.error('Error in planPath:', error);
      throw error;
    }
  };

function App() {
  const [selectedNode, setSelectedNode] = useState<NodeInfo | null>(null);
  const [permanentNodes, setPermanentNodes] = useState<NodeInfo[]>([]);
  const [plannedPath, setPlannedPath] = useState<PathSegment[] | null>(null);
  const [isPlanning, setIsPlanning] = useState(false);

  const addCurrentSelection = () => {
    if (selectedNode) {
      if (!permanentNodes.find(n => n.nodeId === selectedNode.nodeId)) {
        setPermanentNodes([...permanentNodes, selectedNode]);
      }
    }
  };

  const handlePermanentNodeClick = (nodeId: string) => {
    setPermanentNodes(permanentNodes.filter(node => node.nodeId !== nodeId));
    setPlannedPath(null); // Clear planned path when nodes change
  };

  const handlePlanPath = async () => {
    if (permanentNodes.length < 2) {
      alert('Please select at least 2 nodes to plan a path');
      return;
    }

    setIsPlanning(true);
    try {
      const path = await planPath(permanentNodes.map(node => node.position));
      console.log(path.length)
      setPlannedPath(path);
    } catch (error) {
      console.error('Error planning path:', error);
      alert('Error planning path. Please try again.');
    } finally {
      setIsPlanning(false);
    }
  };

  return (
    <div className="p-4">
      {/* Selection Info Display */}
      {selectedNode ? (
        <div className="p-4 mb-4 bg-blue-100 rounded">
          <h3 className="font-bold mb-2">Selected Node</h3>
          <div className="mt-2 flex gap-4">
            <button
              onClick={addCurrentSelection}
              className="px-4 py-1 bg-green-500 text-white rounded hover:bg-green-600"
            >
              Add to Path
            </button>
          </div>
        </div>
      ) : (
        <div className="p-4 mb-4 bg-gray-100 rounded">
          Click on a node to select it
        </div>
      )}
      
      <UserMap 
        center={[51.505, -0.09]}
        zoom={13}
        permanentNodes={permanentNodes}
        onNodeSelect={setSelectedNode}
        onPermanentNodeClick={handlePermanentNodeClick}
        plannedPath={plannedPath}
      />

      {/* Permanent Nodes List and Plan Path Button */}
      <div className="mt-4 p-4 bg-gray-100 rounded">
        <div className="flex justify-between items-center mb-4">
          <h3 className="font-bold">Selected Path Nodes ({permanentNodes.length})</h3>
          <button
            onClick={handlePlanPath}
            disabled={isPlanning || permanentNodes.length < 2}
            className={`px-4 py-2 rounded text-white ${
              isPlanning || permanentNodes.length < 2
                ? 'bg-gray-400 cursor-not-allowed'
                : 'bg-blue-500 hover:bg-blue-600'
            }`}
          >
            {isPlanning ? 'Planning...' : 'Plan Path'}
          </button>
        </div>
        <div className="space-y-2">
          {permanentNodes.map((node) => (
            <div key={node.nodeId} className="flex justify-between items-center">
              <span>Node {node.nodeId}</span>
              <button
                onClick={() => handlePermanentNodeClick(node.nodeId)}
                className="px-2 py-1 text-sm bg-red-500 text-white rounded hover:bg-red-600"
              >
                Remove
              </button>
            </div>
          ))}
        </div>
      </div>
    </div>
  );
}

export default App;
