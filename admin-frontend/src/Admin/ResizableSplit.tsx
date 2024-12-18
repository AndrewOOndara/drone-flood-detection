import React, { MouseEventHandler, useState } from 'react';
import './ResizableSplit.css';

function ResizableSplit(props: {
  left: React.ReactNode,
  right: React.ReactNode
}) {
  const {left, right} = props;
  // Initial split position (50% by default)
  const [splitPosition, setSplitPosition] = useState(50);

  // Mouse down handler for resize bar
  const handleMouseDown: MouseEventHandler = React.useCallback((e) => {
    // Get initial mouse position
    const initialMousePosition = e.clientX;

    // Get initial split position
    const initialSplitPosition = splitPosition;

    // Define mouse move handler
    const handleMouseMove = (e: MouseEvent) => {
      // Calculate new split position
      const newSplitPosition = initialSplitPosition + (e.clientX - initialMousePosition) * 100.0 / window.innerWidth;

      // Update split position
      setSplitPosition(newSplitPosition);
    };

    // Define mouse up handler
    const handleMouseUp = () => {
      // Remove event listeners
      document.removeEventListener('mousemove', handleMouseMove);
      document.removeEventListener('mouseup', handleMouseUp);
    };

    // Add event listeners
    document.addEventListener('mousemove', handleMouseMove);
    document.addEventListener('mouseup', handleMouseUp);
  }, [setSplitPosition]);

  return (
    <div className="resizable-split">
      {/* Left section */}
      <div className="left-section" style={{ width: `${splitPosition}%` }}>
        {left}
      </div>

      {/* Resize bar */}
      <div className="resize-bar" onMouseDown={handleMouseDown} />

      {/* Right section */}
      <div className="right-section" style={{ width: `${100 - splitPosition}%` }}>
        {right}
      </div>
    </div>
  );
};

export default ResizableSplit;