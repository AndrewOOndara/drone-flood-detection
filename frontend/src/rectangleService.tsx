export interface Rectangle {
    id: string;
    bounds: [number, number, number, number]; // [x0, x1, y0, y1]
    name: string;  // optional metadata
  }
  
  export const fetchRectangles = async (): Promise<Rectangle[]> => {
    // Simulate API delay
    await new Promise(resolve => setTimeout(resolve, 800));
    
    // Mock API response
    return [
      {
        id: 'rect-1',
        bounds: [-0.09, -0.08, 51.5, 51.51],
        name: 'Restricted Zone A'
      },
      {
        id: 'rect-2',
        bounds: [-0.11, -0.09, 51.49, 51.505],
        name: 'Restricted Zone B'
      },
      {
        id: 'rect-3',
        bounds: [-0.13, -0.12, 51.51, 51.52],
        name: 'Restricted Zone C'
      }
    ];
  };