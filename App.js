import React, { useState } from 'react';
import './App.css';
import VideoStream from './Pages/VideoStream';
import SensorData from './Pages/SensorData';
import Login from './Pages/Login';

function App() {
    const [loggedIn, setLoggedIn] = useState(false);

    const handleLogin = () => {
        setLoggedIn(true);
    };

    return (
        <div className="App">
            <header className="App-header">
                {!loggedIn && <Login onLogin={handleLogin} />}
                {loggedIn && (
                    <>
                        <VideoStream />
                        <SensorData />
                    </>
                )}
            </header>
        </div>
    );
}

export default App;
