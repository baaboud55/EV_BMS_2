
class AdvancedBMSMonitor {
    constructor() {
        this.websocket = null;
        this.charts = {};
        this.maxDataPoints = 50;
        this.reconnectInterval = 5000;
        this.isConnected = false;

        // Cell voltage colors (8 different colors)
        this.cellColors = [
            '#FF6B6B', '#4ECDC4', '#45B7D1', '#96CEB4',
            '#FFEAA7', '#DDA0DD', '#98D8C8', '#F7DC6F'
        ];

        this.init();
    }

    init() {
        this.setupWebSocket();
        this.setupCharts();
        this.setupEventListeners();

        setTimeout(() => {
            if (!this.isConnected) {
                this.fetchInitialData();
            }
        }, 3000);
    }

    setupWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;

        try {
            this.websocket = new WebSocket(wsUrl);

            this.websocket.onopen = () => {
                console.log('WebSocket connected');
                this.isConnected = true;
                this.updateConnectionStatus(true);
            };

            this.websocket.onmessage = (event) => {
                try {
                    const data = JSON.parse(event.data);
                    this.updateDisplay(data);
                    this.updateCharts(data);
                } catch (error) {
                    console.error('Error parsing WebSocket data:', error);
                }
            };

            this.websocket.onclose = () => {
                console.log('WebSocket disconnected');
                this.isConnected = false;
                this.updateConnectionStatus(false);
                this.scheduleReconnect();
            };

            this.websocket.onerror = (error) => {
                console.error('WebSocket error:', error);
                this.isConnected = false;
                this.updateConnectionStatus(false);
            };

        } catch (error) {
            console.error('Error setting up WebSocket:', error);
            this.updateConnectionStatus(false);
        }
    }

    scheduleReconnect() {
        setTimeout(() => {
            if (!this.isConnected) {
                console.log('Attempting to reconnect...');
                this.setupWebSocket();
            }
        }, this.reconnectInterval);
    }

    updateConnectionStatus(connected) {
        const statusIndicator = document.getElementById('statusIndicator');
        const statusText = document.getElementById('statusText');

        if (connected) {
            statusIndicator.classList.add('connected');
            statusText.textContent = 'Connected';
        } else {
            statusIndicator.classList.remove('connected');
            statusText.textContent = 'Disconnected';
        }
    }

    setupCharts() {
        // Setup individual cell voltages chart (multi-line)
        this.setupCellVoltagesChart();

        // Setup other charts
        const chartConfigs = {
            voltageChart: {
                label: 'Pack Voltage (V)',
                borderColor: '#3498db',
                backgroundColor: 'rgba(52, 152, 219, 0.1)',
                yAxisLabel: 'Voltage (V)'
            },
            currentChart: {
                label: 'Current (A)',
                borderColor: '#e74c3c',
                backgroundColor: 'rgba(231, 76, 60, 0.1)',
                yAxisLabel: 'Current (A)'
            },
            socChart: {
                label: 'State of Charge (%)',
                borderColor: '#27ae60',
                backgroundColor: 'rgba(39, 174, 96, 0.1)',
                yAxisLabel: 'SOC (%)',
                suggestedMin: 0,
                suggestedMax: 100
            },
            sohChart: {
                label: 'State of Health (%)',
                borderColor: '#f39c12',
                backgroundColor: 'rgba(243, 156, 18, 0.1)',
                yAxisLabel: 'SOH (%)',
                suggestedMin: 0,
                suggestedMax: 100
            },
        };

        Object.keys(chartConfigs).forEach(chartId => {
            const config = chartConfigs[chartId];
            const ctx = document.getElementById(chartId);
            if (!ctx) return;

            this.charts[chartId] = new Chart(ctx.getContext('2d'), {
                type: 'line',
                data: {
                    datasets: [{
                        label: config.label,
                        data: [],
                        borderColor: config.borderColor,
                        backgroundColor: config.backgroundColor,
                        borderWidth: 2,
                        fill: true,
                        tension: 0.4,
                        pointRadius: 0,
                        pointHoverRadius: 5
                    }]
                },
                options: {
                    responsive: true,
                    maintainAspectRatio: false,
                    interaction: {
                        intersect: false,
                        mode: 'index'
                    },
                    plugins: {
                        legend: {
                            display: false
                        }
                    },
                    scales: {
                        x: {
                            type: 'time',
                            time: {
                                unit: 'second',
                                stepSize: 10,
                                displayFormats: {
                                    second: 'HH:mm:ss',
                                    minute: 'HH:mm',
                                    hour: 'HH:mm'
                                },
                                tooltipFormat: 'HH:mm:ss'
                            },
                            grid: {
                                color: 'rgba(0, 0, 0, 0.1)'
                            }
                        },
                        y: {
                            beginAtZero: false,
                            suggestedMin: config.suggestedMin,
                            suggestedMax: config.suggestedMax,
                            grid: {
                                color: 'rgba(0, 0, 0, 0.1)'
                            },
                            title: {
                                display: true,
                                text: config.yAxisLabel
                            }
                        }
                    },
                    animation: {
                        duration: 0
                    }
                }
            });
        });
    }

    setupCellVoltagesChart() {
        const ctx = document.getElementById('cellVoltagesChart');
        if (!ctx) return;

        // Create datasets for 8 cells
        const datasets = [];
        for (let i = 0; i < 8; i++) {
            datasets.push({
                label: `Cell ${i + 1}`,
                data: [],
                borderColor: this.cellColors[i],
                backgroundColor: this.cellColors[i] + '20',
                borderWidth: 2,
                fill: false,
                tension: 0.4,
                pointRadius: 0,
                pointHoverRadius: 5
            });
        }

        this.charts.cellVoltagesChart = new Chart(ctx.getContext('2d'), {
            type: 'line',
            data: {
                datasets: datasets
            },
            options: {
                responsive: true,
                maintainAspectRatio: false,
                interaction: {
                    intersect: false,
                    mode: 'index'
                },
                plugins: {
                    legend: {
                        display: true,
                        position: 'top',
                        labels: {
                            boxWidth: 20,
                            padding: 15
                        }
                    }
                },
                scales: {
                    x: {
                        type: 'time',
                        time: {
                            unit: 'second',
                            stepSize: 10,
                            displayFormats: {
                                second: 'HH:mm:ss',
                                minute: 'HH:mm',
                                hour: 'HH:mm'
                            },
                            tooltipFormat: 'HH:mm:ss'
                        },
                        grid: {
                            color: 'rgba(0, 0, 0, 0.1)'
                        }
                    },
                    y: {
                        beginAtZero: false,
                        suggestedMin: 3.0,
                        suggestedMax: 4.2,
                        grid: {
                            color: 'rgba(0, 0, 0, 0.1)'
                        },
                        title: {
                            display: true,
                            text: 'Cell Voltage (V)'
                        }
                    }
                },
                animation: {
                    duration: 0
                }
            }
        });
    }

    updateDisplay(data) {
        // Update basic values
        document.getElementById('packVoltage').textContent = data.packVoltage?.toFixed(2) || '--';
        document.getElementById('current').textContent = data.current?.toFixed(2) || '--';
        document.getElementById('soc').textContent = data.soc?.toFixed(1) || '--';
        document.getElementById('soh').textContent = data.soh?.toFixed(1) || '--';

        // Update battery state with styling
        const batteryStateElement = document.getElementById('batteryState');
        if (data.batteryState) {
            batteryStateElement.textContent = data.batteryState;
            batteryStateElement.className = 'battery-state-value ' + data.batteryState.toLowerCase();
        }

        // Update cumulative capacity
        document.getElementById('cumulativeCapacity').textContent = data.cumulativeCapacity?.toFixed(2) || '--';

        // Update progress bars
        if (data.soc !== undefined) {
            const socProgress = document.getElementById('socProgress');
            socProgress.style.width = `${Math.max(0, Math.min(100, data.soc))}%`;
        }

        if (data.soh !== undefined) {
            const sohProgress = document.getElementById('sohProgress');
            sohProgress.style.width = `${Math.max(0, Math.min(100, data.soh))}%`;
        }

        // Update slave temperatures
        if (data.slaveTemperatures && Array.isArray(data.slaveTemperatures)) {
            for (let i = 0; i < 4; i++) {
                const tempElement = document.getElementById(`temp${i + 1}`);
                if (tempElement && data.slaveTemperatures[i] !== undefined) {
                    tempElement.textContent = `${data.slaveTemperatures[i].toFixed(1)}°C`;
                }
            }
        }

        // Update protection and control indicators
        this.updateIndicator('prechargeLed', data.prechargeActive);
        this.updateIndicator('protectionLed', data.protectionPMOSActive);
        this.updateIndicator('balancingLed', data.activeBalancingActive);

        // Update fault indicators
        this.updateFaultIndicator('overvoltageAlarm', data.overvoltage);
        this.updateFaultIndicator('undervoltageAlarm', data.undervoltage);
        this.updateFaultIndicator('overcurrentAlarm', data.overcurrent);
    }

    updateIndicator(elementId, isActive) {
        const element = document.getElementById(elementId);
        if (element) {
            if (isActive) {
                element.classList.add('active');
            } else {
                element.classList.remove('active');
            }
        }
    }

    updateFaultIndicator(elementId, isFault) {
        const element = document.getElementById(elementId);
        if (element) {
            if (isFault) {
                element.classList.add('active');
            } else {
                element.classList.remove('active');
            }
        }
    }

    updateCharts(data) {
        // data.timestamp is already in seconds (Unix epoch), convert to milliseconds for JavaScript Date
        const timestamp = new Date(data.timestamp * 1000);


        // Update cell voltages chart
        if (this.charts.cellVoltagesChart && data.cellVoltages && Array.isArray(data.cellVoltages)) {
            const chart = this.charts.cellVoltagesChart;

            for (let i = 0; i < 8 && i < data.cellVoltages.length; i++) {
                const dataset = chart.data.datasets[i];
                if (dataset) {
                    dataset.data.push({
                        x: timestamp,
                        y: data.cellVoltages[i]
                    });

                    if (dataset.data.length > this.maxDataPoints) {
                        dataset.data.shift();
                    }
                }
            }

            chart.update('none');
        }

        // Update other charts
        const chartData = {
            voltageChart: data.packVoltage,
            currentChart: data.current,
            socChart: data.soc,
            sohChart: data.soh,
        };

        Object.keys(chartData).forEach(chartId => {
            if (this.charts[chartId] && chartData[chartId] !== undefined) {
                const chart = this.charts[chartId];
                const dataset = chart.data.datasets[0];

                dataset.data.push({
                    x: timestamp,
                    y: chartData[chartId]
                });

                if (dataset.data.length > this.maxDataPoints) {
                    dataset.data.shift();
                }

                chart.update('none');
            }
        });
    }

    async fetchInitialData() {
        try {
            const response = await fetch('/api/bms');
            if (response.ok) {
                const data = await response.json();
                this.updateDisplay(data);
                this.updateCharts(data);
                console.log('Initial data loaded via HTTP');
            }
        } catch (error) {
            console.error('Error fetching initial data:', error);
        }
    }

    setupEventListeners() {
        document.addEventListener('visibilitychange', () => {
            if (document.hidden) {
                console.log('Page hidden');
            } else {
                console.log('Page visible');
                if (!this.isConnected) {
                    this.setupWebSocket();
                }
            }
        });

        window.addEventListener('beforeunload', () => {
            if (this.websocket) {
                this.websocket.close();
            }
        });
    }
}

// Initialize the Advanced BMS Monitor when the page loads
document.addEventListener('DOMContentLoaded', () => {
    window.advancedBmsMonitor = new AdvancedBMSMonitor();
});

// Export for potential external use
if (typeof module !== 'undefined' && module.exports) {
    module.exports = AdvancedBMSMonitor;
}