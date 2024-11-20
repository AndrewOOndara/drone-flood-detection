import React, { useState } from 'react';
import Admin from './Admin';
import User from './User';

function App() {
  const [isAdmin, setIsAdmin] = useState(false);
  const inner = isAdmin ? <Admin /> : <User />;
  return (
    <div>
      <input
        type="checkbox"
        checked={isAdmin}
        onChange={() => {
            setIsAdmin(!isAdmin);
        }}
      />
      {inner}
    </div>
  )
};

export default App;

