import React, { useEffect, useState } from 'react';
import io from 'socket.io-client';
import './VideoStream.css';
import SensorData from './SensorData';  // Import SensorData

const socket = io('https://quest-robot-app.eu.ngrok.io', {
  transports: ['websocket'],
  reconnectionAttempts: 5,
  reconnectionDelay: 1000,
  timeout: 20000,
});

function VideoStream() {
  const [frame, setFrame] = useState(null);

  useEffect(() => {
    socket.on('connect', () => {
      console.log('Connected to WebSocket server');
    });

    socket.on('hello', (arg) => {
      console.log('Received hello:', arg);
    });

    socket.on('video_frame', (data) => {
      console.log('Received video frame');
      const imageSrc = data:image/jpeg;base64,${data.frame};
      setFrame(imageSrc);
    });

    socket.on('disconnect', (reason) => {
      console.log('Disconnected from WebSocket server:', reason);
    });

    socket.on('reconnect_attempt', () => {
      console.log('Reconnecting...');
    });

    socket.on('reconnect_failed', () => {
      console.log('Reconnection failed');
    });

    return () => {
      socket.off('hello');
      socket.off('video_frame');
      socket.off('connect');
      socket.off('disconnect');
      socket.off('reconnect_attempt');
      socket.off('reconnect_failed');
    };
  }, []);

  return (
    <div className="VideoStream">
      <header className="VideoStream-header">
        <h1>Live Video Stream</h1>
        {frame ? <img src={frame} alt="Live Stream" /> : <p>Loading...</p>}
      </header>
      <SensorData />  {/* Add SensorData here */}
    </div>
  );
}

export default VideoStream;
