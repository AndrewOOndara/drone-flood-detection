import React, { useState } from 'react';
import MapComponent from './MapComponent';
import { AppSettings, LatLon, NodeId } from '../types';

type Mode = 'node' | 'edge';

type NodeInfo = {
  nodeId: string;
  position: [number, number];
}

type EdgeInfo = {
  fromNodeId: string;
  toNodeId: string;
  fromPosition: [number, number];
  toPosition: [number, number];
}

type Props = Pick<AppSettings, "map_lat_lon" | "map_radius" | "edges_to_visit"> & {
  updateSettings: (arg: Partial<AppSettings>) => void
}

function EditCoverageMap(props: Props) {
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
    <section className="coverage-editor">
      {/* Mode Toggle */}
      <div className="">
        <label className="">
          <input
            type="checkbox"
            className=""
            checked={mode === 'edge'}
            onChange={() => {
              setMode(mode === 'node' ? 'edge' : 'node');
              setSelectedNode(null);
              setSelectedEdge(null);
            }}
          />
          Edge Select Mode
        </label>
      </div>


      {/* Selection Info Display */}
      {/* {mode === 'node' && selectedNode ? (
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
      )} */}
      
      <MapComponent 
        center={props.map_lat_lon}
        radius={props.map_radius}
        mode={mode}
        permanentNodes={permanentNodes}
        onNodeSelect={setSelectedNode}
        onEdgeSelect={setSelectedEdge}
        onPermanentNodeClick={handlePermanentNodeClick}
      />

      {/* Permanent Nodes List */}
      <div className="">
        <h3 className="">Selected Path Nodes ({permanentNodes.length})</h3>
        <div className="">
          {permanentNodes.map((node) => (
            <div key={node.nodeId} className="">
              <span>Node {node.nodeId}</span>
              <button
                onClick={() => handlePermanentNodeClick(node.nodeId)}
                className=""
              >
                Remove
              </button>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

export default EditCoverageMap;