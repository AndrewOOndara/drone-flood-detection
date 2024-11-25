'use client';

import React, { useEffect, useRef } from 'react';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';

interface UserMapProps {
  center: [number, number];
  zoom: number;
  permanentNodes: { nodeId: string; position: [number, number] }[];
  onNodeSelect: (node: { nodeId: string; position: [number, number] } | null) => void;
  onPermanentNodeClick: (nodeId: string) => void;
  plannedPath: { fromNode: { position: [number, number] }; toNode: { position: [number, number] } }[] | null;
}

const UserMap: React.FC<UserMapProps> = ({
  center,
  zoom,
  permanentNodes,
  onNodeSelect,
  onPermanentNodeClick,
  plannedPath,
}) => {
  const mapRef = useRef<L.Map | null>(null);

  useEffect(() => {
    // Ensure map is only initialized once
    if (!mapRef.current) {
      mapRef.current = L.map('map').setView(center, zoom);

      // Add tile layer
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors',
      }).addTo(mapRef.current);

      // Click event to add nodes
      mapRef.current.on('click', (event: L.LeafletMouseEvent) => {
        const { lat, lng } = event.latlng;
        const nodeId = `${lat.toFixed(5)},${lng.toFixed(5)}`; // Generate a mock ID
        onNodeSelect({ nodeId, position: [lat, lng] });
      });
    }

    // Cleanup on unmount
    return () => {
      if (mapRef.current) {
        mapRef.current.off();
        mapRef.current.remove();
        mapRef.current = null;
      }
    };
  }, [center, zoom, onNodeSelect]);

  useEffect(() => {
    // Clear existing layers
    if (mapRef.current) {
      mapRef.current.eachLayer((layer) => {
        if ((layer as L.TileLayer).options.attribution !== undefined) {
          // Retain the tile layer
          return;
        }
        mapRef.current?.removeLayer(layer);
      });

      // Add permanent nodes
      permanentNodes.forEach((node) => {
        const marker = L.marker(node.position).addTo(mapRef.current!);
        marker.on('click', () => onPermanentNodeClick(node.nodeId));
      });

      // Add planned path
      if (plannedPath) {
        plannedPath.forEach(({ fromNode, toNode }) => {
          L.polyline([fromNode.position, toNode.position], { color: 'blue' }).addTo(mapRef.current!);
        });
      }
    }
  }, [permanentNodes, plannedPath, onPermanentNodeClick]);

  return <div id="map" style={{ height: '500px', width: '100%' }} />;
};

export default UserMap;
