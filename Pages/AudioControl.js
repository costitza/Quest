import React, { useState } from 'react';
import axios from 'axios';
import './AudioControl.css';

function AudioControl() {
    const [audioActive, setAudioActive] = useState(false);
    const [audioElement, setAudioElement] = useState(null);

    const startAudio = async () => {
        try {
            await axios.post('http://localhost:5000/start_audio', {}, { withCredentials: true });
            console.log('Audio stream started');
            
            const audio = new Audio('http://localhost:5000/audio_stream');
            audio.play();
            setAudioElement(audio);
            setAudioActive(true);
        } catch (error) {
            console.error('Failed to start audio stream', error);
        }
    };

    const stopAudio = async () => {
        try {
            await axios.post('http://localhost:5000/stop_audio', {}, { withCredentials: true });
            console.log('Audio stream stopped');
            
            if (audioElement) {
                audioElement.pause();
                setAudioElement(null);
            }
            setAudioActive(false);
        } catch (error) {
            console.error('Failed to stop audio stream', error);
        }
    };

    return (
        <div className="AudioControl">
            <button onClick={audioActive ? stopAudio : startAudio}>
                {audioActive ? 'Stop Audio' : 'Start Audio'}
            </button>
        </div>
    );
}

export default AudioControl;
