import React from 'react';
import './VideoStream.css';

function VideoStream() {
    return (
        <div className="VideoStream">
            <h2>Live Video Feed</h2>
            <img src="http://localhost:5000/video_feed" alt="Video Feed" />
        </div>
    );
}

export default VideoStream;
