import React, { useState } from 'react';
import MapComponent from './Map';

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

function User() {
  const [mode, setMode] = useState<Mode>('edge');
  const [selectedNode, setSelectedNode] = useState<NodeInfo | null>(null);
  const [selectedEdge, setSelectedEdge] = useState<EdgeInfo | null>(null);
  const [permanentNodes, setPermanentNodes] = useState<NodeInfo[]>([]);

  const addCurrentSelection = () => {
    if (mode === 'node' && selectedNode) {
      if (!permanentNodes.find(n => n.nodeId === selectedNode.nodeId)) {
        setPermanentNodes([...permanentNodes, selectedNode]);
      }
    } else if (mode === 'edge' && selectedEdge) {
      const newNodes: NodeInfo[] = [];
      
      // Add start node if not already present
      if (!permanentNodes.find(n => n.nodeId === selectedEdge.fromNodeId)) {
        newNodes.push({
          nodeId: selectedEdge.fromNodeId,
          position: selectedEdge.fromPosition
        });
      }
      
      // Add end node if not already present
      if (!permanentNodes.find(n => n.nodeId === selectedEdge.toNodeId)) {
        newNodes.push({
          nodeId: selectedEdge.toNodeId,
          position: selectedEdge.toPosition
        });
      }
      
      if (newNodes.length > 0) {
        setPermanentNodes([...permanentNodes, ...newNodes]);
      }
    }
  };

  const handlePermanentNodeClick = (nodeId: string) => {
    setPermanentNodes(permanentNodes.filter(node => node.nodeId !== nodeId));
  };

  return (
    <div className="p-4">
      {/* Mode Toggle */}
      <div className="mb-4">
        <label className="inline-flex items-center relative cursor-pointer">
          <input
            type="checkbox"
            className="sr-only peer"
            checked={mode === 'edge'}
            onChange={() => {
              setMode(mode === 'node' ? 'edge' : 'node');
              setSelectedNode(null);
              setSelectedEdge(null);
            }}
          />
          <div className="w-36 h-10 bg-gray-200 rounded-full peer peer-checked:after:translate-x-[6.5rem] after:content-[''] after:absolute after:top-0.5 after:left-0.5 after:bg-white after:rounded-full after:h-9 after:w-16 after:transition-all peer-checked:bg-blue-500"></div>
          <span className="absolute right-4 text-sm font-medium text-gray-800 peer-checked:text-white">Edge Select Mode</span>
        </label>
      </div>


      {/* Selection Info Display */}
      {mode === 'node' && selectedNode ? (
        <div className="p-4 mb-4 bg-blue-100 rounded">
          <h3 className="font-bold mb-2">Selected Node</h3>
          <p>Node ID: {selectedNode.nodeId}</p>
          <p>Position: ({selectedNode.position.join(', ')})</p>
          <div className="mt-2 flex gap-4">
            <a 
              href={`https://www.openstreetmap.org/node/${selectedNode.nodeId}`}
              target="_blank"
              rel="noopener noreferrer"
              className="text-blue-600 hover:underline"
            >
              View Node on OSM
            </a>
            <button
              onClick={addCurrentSelection}
              className="px-4 py-1 bg-green-500 text-white rounded hover:bg-green-600"
            >
              Add to Path
            </button>
          </div>
        </div>
      ) : mode === 'edge' && selectedEdge ? (
        <div className="p-4 mb-4 bg-blue-100 rounded">
          <h3 className="font-bold mb-2">Selected Edge</h3>
          <p>From Node ID: {selectedEdge.fromNodeId} ({selectedEdge.fromPosition.join(', ')})</p>
          <p>To Node ID: {selectedEdge.toNodeId} ({selectedEdge.toPosition.join(', ')})</p>
          <div className="mt-2 flex gap-4">
            <div>
              <a 
                href={`https://www.openstreetmap.org/node/${selectedEdge.fromNodeId}`}
                target="_blank"
                rel="noopener noreferrer"
                className="text-blue-600 hover:underline mr-4"
              >
                View Start Node on OSM
              </a>
              <a 
                href={`https://www.openstreetmap.org/node/${selectedEdge.toNodeId}`}
                target="_blank"
                rel="noopener noreferrer"
                className="text-blue-600 hover:underline"
              >
                View End Node on OSM
              </a>
            </div>
            <button
              onClick={addCurrentSelection}
              className="px-4 py-1 bg-green-500 text-white rounded hover:bg-green-600"
            >
              Add Edge Nodes to Path
            </button>
          </div>
        </div>
      ) : (
        <div className="p-4 mb-4 bg-gray-100 rounded">
          Click on a {mode === 'node' ? 'node' : 'road'} to select it
        </div>
      )}
      
      <MapComponent 
        center={[51.505, -0.09]}
        zoom={13}
        mode={mode}
        permanentNodes={permanentNodes}
        onNodeSelect={setSelectedNode}
        onEdgeSelect={setSelectedEdge}
        onPermanentNodeClick={handlePermanentNodeClick}
      />

      {/* Permanent Nodes List */}
      <div className="mt-4 p-4 bg-gray-100 rounded">
        <h3 className="font-bold mb-2">Selected Path Nodes ({permanentNodes.length})</h3>
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

export default User;