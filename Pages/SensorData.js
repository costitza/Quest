import React, { useState, useEffect } from 'react';
import axios from 'axios';
import './SensorData.css';
import { WiHumidity, WiThermometer } from 'react-icons/wi'; // Import icons from react-icons library

function SensorData() {
    const [sensorData, setSensorData] = useState({
        front: '--',
        right: '--',
        left: '--',
        back: '--',
        temperature: '--',
        humidity: '--'
    });

    useEffect(() => {
        const interval = setInterval(() => {
            fetchSensorData();
        }, 50); // Fetch sensor data every second

        return () => clearInterval(interval);
    }, []);

    const fetchSensorData = async () => {
        try {
            const response = await axios.get('http://localhost:5000/sensor_data', { withCredentials: true });
            if (response.data && !response.data.error) {
                const { front, right, left, back, temperature, humidity } = response.data;
                setSensorData({
                    front: front,
                    right: right,
                    left: left,
                    back: back,
                    temperature: temperature.toFixed(1),
                    humidity: humidity.toFixed(1)
                });
            } else {
                console.error('Error fetching sensor data:', response.data.error);
            }
        } catch (error) {
            console.error('Error fetching sensor data:', error);
        }
    };

    return (
        <div className="SensorData">
            <div className="top-right">
                <div className="sensor-item">
                    <WiHumidity className="icon" />
                    <p>{sensorData.humidity} %</p>
                </div>
                <div className="sensor-item">
                    <WiThermometer className="icon" />
                    <p>{sensorData.temperature} °C</p>
                </div>
            </div>
            <div className="radar-container">
                <div className="car-layout">
                    <div className="car-sensor front-sensor">
                        <p>{sensorData.front} cm</p>
                    </div>
                    <div className="car-sensor right-sensor">
                        <p>{sensorData.right} cm</p>
                    </div>
                    <div className="car-sensor left-sensor">
                        <p>{sensorData.left} cm</p>
                    </div>
                    <div className="car-sensor back-sensor">
                        <p>{sensorData.back} cm</p>
                    </div>
                    <div className="device-icon"></div>
                </div>
            </div>
        </div>
    );
}

export default SensorData;
