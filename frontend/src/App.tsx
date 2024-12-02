import React, { useState } from 'react';
import Admin from './Admin/';



function App() {
  const [isAdmin, setIsAdmin] = useState(false);
  const inner = <Admin />;
  return (
    <React.Fragment>
      <div className="app-title-row">
        <h1 className="app-title">{isAdmin ? "HPE Admin Controls" : "HPE"}</h1>
        <label className="admin-user-switch-label">
          <input
            type="checkbox"
            id="admin-user-switch"
            className="admin-user-switch"
            checked={isAdmin}
            onChange={() => {
              setIsAdmin(!isAdmin);
            }}
          />
          {isAdmin ? "🚶" : "👩‍💻"}
        </label>
      </div>
      {inner}
    </React.Fragment>
  )
};

export default App;

