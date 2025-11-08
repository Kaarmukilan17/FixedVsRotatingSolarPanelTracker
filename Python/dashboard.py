"""
Solar Tracker Power Comparison Dashboard
Modular GUI for comparing rotating vs fixed solar panel performance
Author: Scientific Visualization Expert
"""

import sys
import random
import csv
import time
import math
from datetime import datetime
from typing import List, Dict, Optional, Tuple
from collections import deque
from dataclasses import dataclass

# Try importing required packages with fallbacks
try:
    from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                                QWidget, QLabel, QComboBox, QPushButton, QProgressBar,
                                QGroupBox, QStatusBar, QMessageBox, QFrame, QGridLayout,QFileDialog)
    from PyQt5.QtCore import QTimer, QThread, pyqtSignal, Qt, QSettings, QRect
    #from PyQt5.QtGui import QFont, QPalette, QColor, QPainter, QLinearGradient
    from PyQt5.QtGui import QFont, QPalette, QColor, QPainter, QLinearGradient, QPen

    PYQT5_AVAILABLE = True
except ImportError:
    PYQT5_AVAILABLE = False

try:
    import pyqtgraph as pg
    PYQTGRAPH_AVAILABLE = True
except ImportError:
    PYQTGRAPH_AVAILABLE = False

try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

class Config:
    """Central configuration for the application"""
    
    # Serial communication
    BAUD_RATE = 115200
    DATA_PREFIX = ""
    MAX_POINTS = 500
    UPDATE_INTERVAL_MS = 200
    AUTO_RECONNECT_INTERVAL_MS = 5000
    
    # Data simulation (when no Arduino connected)
    SIMULATION_RANGES = {
        'LDR1': (200, 1000),
        'LDR2': (200, 1000), 
        'ServoAngle': (0, 180),
        'V_fixed': (3.0, 5.0),
        'I_fixed': (50, 200),
        'V_rot': (3.5, 5.5),
        'I_rot': (80, 250)
    }
    
    # UI colors (dark theme)
    COLORS = {
        'background': '#2b2b2b',
        'foreground': '#ffffff',
        'accent': '#1e90ff',
        'success': '#00ff00',
        'warning': '#ffff00',
        'error': '#ff0000',
        'fixed_panel': '#ff4444',
        'rotating_panel': '#44ff44',
        'gain_positive': '#00ff00',
        'gain_negative': '#ff0000'
    }
    
    # Plot configuration
    PLOT_TIME_WINDOW = 60  # seconds
    PLOT_UPDATE_RATE = 10  # Hz
    
    # File paths
    CSV_LOG_PREFIX = "solar_tracker"
    SETTINGS_FILE = "solar_tracker_settings.ini"


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class SolarData:
    """Data structure for solar tracker measurements"""
    timestamp: float
    LDR1: float
    LDR2: float
    ServoAngle: float
    V_fixed: float
    I_fixed: float
    P_fixed: float
    V_rot: float
    I_rot: float
    P_rot: float
    
    @property
    def power_gain(self) -> float:
        """Calculate power gain percentage"""
        if self.P_fixed > 0:
            return ((self.P_rot - self.P_fixed) / self.P_fixed) * 100
        return 0.0
    
    @classmethod
    def from_csv_line(cls, line: str) -> Optional['SolarData']:
        """Create SolarData from CSV line"""
        try:
            parts = line.strip().split(',')
            if len(parts) >= 10:
                return cls(
                    timestamp=float(parts[0]),
                    LDR1=float(parts[1]),
                    LDR2=float(parts[2]),
                    ServoAngle=float(parts[3]),
                    V_fixed=float(parts[4]),
                    I_fixed=float(parts[5]),
                    P_fixed=float(parts[6]),
                    V_rot=float(parts[7]),
                    I_rot=float(parts[8]),
                    P_rot=float(parts[9])
                )
        except (ValueError, IndexError):
            pass
        return None


# =============================================================================
# CORE DATA MANAGEMENT
# =============================================================================

class DataBuffer:
    """Manages rolling buffer of solar data with validation"""
    
    def __init__(self, max_points: int = Config.MAX_POINTS):
        self.max_points = max_points
        self.data_buffer = deque(maxlen=max_points)
        self._last_valid_data = None
        
    def add_data(self, data: SolarData) -> bool:
        """Add validated data to buffer"""
        if self._validate_data(data):
            self.data_buffer.append(data)
            self._last_valid_data = data
            print(f"✅ Data added to buffer: LDR1={data.LDR1}, Angle={data.ServoAngle}")  # Debug
            return True
        else:
            print(f"❌ Data validation failed: LDR1={data.LDR1}, Angle={data.ServoAngle}")  # Debug
            return False
    
    def _validate_data(self, data: SolarData) -> bool:
        """Validate data ranges with more tolerance for Arduino data"""
        try:
            # Relax LDR range check (Arduino analogRead 0-1023)
            if not (0 <= data.LDR1 <= 1024 and 0 <= data.LDR2 <= 1024):
                return False
                
            # Relax servo angle check  
            if not (0 <= data.ServoAngle <= 180):
                return False
                
            # Allow negative currents (INA219 can report negative)
            if not (data.V_fixed >= 0 and data.V_rot >= 0):
                return False
                
            # Remove strict power calculation check - Arduino uses abs()
            # The Arduino calculates P_fixed = abs(V_fixed*I_fixed) which may not match exactly
            # due to floating point precision and absolute value conversion
            
            return True
            
        except (TypeError, ValueError):
            return False
    
    def get_recent_data(self, count: int = 1) -> List[SolarData]:
        """Get most recent data points"""
        return list(self.data_buffer)[-count:]
    
    def get_all_data(self) -> List[SolarData]:
        """Get all data in buffer"""
        return list(self.data_buffer)
    
    def clear(self):
        """Clear all data"""
        self.data_buffer.clear()
        self._last_valid_data = None
    
    @property
    def last_valid_data(self) -> Optional[SolarData]:
        """Get last valid data point"""
        return self._last_valid_data
    
    @property
    def count(self) -> int:
        """Get current data point count"""
        return len(self.data_buffer)


class DataSimulator:
    """Generates realistic simulated data for testing"""
    
    def __init__(self):
        self._last_values = {}
        self._time_offset = time.time()
        
    def generate_sample(self) -> SolarData:
        """Generate a realistic solar data sample"""
        current_time = time.time() - self._time_offset
        
        # Simulate sun movement with smooth transitions
        base_angle = 90 + 45 * math.sin(current_time / 30.0)  # 30-second cycle
        
        # Light sensors follow sun position with some noise
        angle_rad = math.radians(base_angle)
        ldr1_base = 500 + 400 * max(0, math.cos(angle_rad - math.pi/4))
        ldr2_base = 500 + 400 * max(0, math.cos(angle_rad + math.pi/4))
        
        # Add some realistic noise
        ldr1 = max(0, ldr1_base + random.gauss(0, 20))
        ldr2 = max(0, ldr2_base + random.gauss(0, 20))
        
        # Servo angle tracks sun position with some lag
        servo_angle = max(0, min(180, base_angle + random.gauss(0, 5)))
        
        # Voltages and currents depend on light intensity
        total_light = (ldr1 + ldr2) / 2
        
        # Fixed panel - less efficient
        v_fixed = 3.5 + (total_light / 1000) * 1.5
        i_fixed = 100 + (total_light / 1000) * 100
        
        # Rotating panel - more efficient due to better alignment
        v_rot = 4.0 + (total_light / 1000) * 1.5
        i_rot = 150 + (total_light / 1000) * 100
        
        # Calculate power
        p_fixed = v_fixed * i_fixed
        p_rot = v_rot * i_rot
        
        return SolarData(
            timestamp=time.time(),
            LDR1=ldr1,
            LDR2=ldr2,
            ServoAngle=servo_angle,
            V_fixed=v_fixed,
            I_fixed=i_fixed,
            P_fixed=p_fixed,
            V_rot=v_rot,
            I_rot=i_rot,
            P_rot=p_rot
        )


# =============================================================================
# SERIAL COMMUNICATION
# =============================================================================

class SerialManager(QThread):
    """Handles serial communication with Arduino in separate thread"""
    
    data_received = pyqtSignal(SolarData)
    status_update = pyqtSignal(str, str)  # status, message
    connection_changed = pyqtSignal(bool)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.is_connected = False
        self.is_running = False
        self.current_port = None
        self.baud_rate = Config.BAUD_RATE
        
    def connect_to_port(self, port_name: str) -> bool:
        """Attempt to connect to specified serial port"""
        if not SERIAL_AVAILABLE:
            self.status_update.emit("error", "pySerial not available")
            return False
            
        try:
            self.disconnect()
            
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=self.baud_rate,
                timeout=1.0
            )
            
            self.current_port = port_name
            self.is_connected = True
            
            # Wait for Arduino to finish calibration (5 seconds)
            self.status_update.emit("info", "Connected, waiting for Arduino calibration...")
            time.sleep(5)
            
            self.connection_changed.emit(True)
            self.status_update.emit("success", f"Connected to {port_name} - Ready for data")
            return True
            
        except serial.SerialException as e:
            self.status_update.emit("error", f"Connection failed: {str(e)}")
            self.is_connected = False
            self.connection_changed.emit(False)
            return False
    
    def disconnect(self):
        """Disconnect from current serial port"""
        self.is_connected = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.serial_port = None
        self.connection_changed.emit(False)
        self.status_update.emit("info", "Disconnected")
    
    def run(self):
        """Main thread loop for reading serial data"""
        self.is_running = True
        buffer = ""
        
        while self.is_running:
            if self.is_connected and self.serial_port and self.serial_port.is_open:
                try:
                    # Read available data
                    if self.serial_port.in_waiting > 0:
                        data = self.serial_port.readline().decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # Process complete lines
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            self._process_line(line.strip())
                            
                except (serial.SerialException, UnicodeDecodeError) as e:
                    self.status_update.emit("error", f"Read error: {str(e)}")
                    self.is_connected = False
                    self.connection_changed.emit(False)
            
            # Small delay to prevent CPU overload
            self.msleep(10)
        
    def _process_line(self, line: str):
        """Process a single line of serial data from Arduino"""
        data_str = line.strip()
        
        # Skip empty lines, calibration messages, and CSV header
        if (not data_str or 
            data_str.startswith("Calibrating") or
            data_str.startswith("Offset:") or
            data_str.startswith("INA219") or
            data_str.startswith("LDR1")):
            # Debug: show what we're skipping
            if data_str:
                self.status_update.emit("debug", f"Skipping: {data_str}")
            return
            
        # Debug: show data being processed
        self.status_update.emit("debug", f"Processing: {data_str}")
            
        try:
            # Parse Arduino's CSV format: LDR1,LDR2,smoothDiff,pos,V_fixed,I_fixed,P_fixed,V_rot,I_rot,P_rot
            values = [float(x) for x in data_str.split(',')]
            
            if len(values) >= 10:  # Arduino sends 10 values
                # Create SolarData object matching Arduino order
                solar_data = SolarData(
                    timestamp=time.time(),
                    LDR1=values[0],      # LDR1
                    LDR2=values[1],      # LDR2  
                    ServoAngle=values[3], # pos (4th value)
                    V_fixed=values[4],   # V_fixed
                    I_fixed=values[5],   # I_fixed
                    P_fixed=values[6],   # P_fixed
                    V_rot=values[7],     # V_rot
                    I_rot=values[8],     # I_rot
                    P_rot=values[9]      # P_rot
                )
                
                # Debug: show parsed data
                self.status_update.emit("debug", f"Parsed: LDR1={solar_data.LDR1}, Angle={solar_data.ServoAngle}")
                
                # Emit data
                self.data_received.emit(solar_data)
                
        except (ValueError, IndexError) as e:
            # Only show warning for lines that look like data but failed to parse
            if data_str and any(c.isdigit() for c in data_str):
                self.status_update.emit("warning", f"Data parsing error: {str(e)} - Line: {data_str}")
        
    def stop(self):
        """Stop the thread"""
        self.is_running = False
        self.disconnect()
        self.wait(1000)


# =============================================================================
# DATA LOGGING
# =============================================================================

class DataLogger:
    """Handles CSV data logging"""
    
    def __init__(self):
        self.is_logging = False
        self.csv_file = None
        self.writer = None
        self.filename = None
        
    def start_logging(self, filename: str = None) -> bool:
        """Start logging to CSV file"""
        if self.is_logging:
            return False
            
        try:
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                self.filename = f"{Config.CSV_LOG_PREFIX}_{timestamp}.csv"
            else:
                self.filename = filename
                
            self.csv_file = open(self.filename, 'w', newline='')
            self.writer = csv.writer(self.csv_file)
            
            # Write header
            self.writer.writerow([
                'timestamp', 'LDR1', 'LDR2', 'ServoAngle',
                'V_fixed', 'I_fixed', 'P_fixed', 
                'V_rot', 'I_rot', 'P_rot', 'PowerGain'
            ])
            
            self.is_logging = True
            return True
            
        except IOError as e:
            print(f"Logging error: {e}")
            return False
    
    def log_data(self, data: SolarData) -> bool:
        """Log a single data point"""
        if not self.is_logging or not self.writer:
            return False
            
        try:
            self.writer.writerow([
                data.timestamp, data.LDR1, data.LDR2, data.ServoAngle,
                data.V_fixed, data.I_fixed, data.P_fixed,
                data.V_rot, data.I_rot, data.P_rot, data.power_gain
            ])
            self.csv_file.flush()
            return True
        except IOError as e:
            print(f"Logging error: {e}")
            return False
    
    def stop_logging(self) -> bool:
        """Stop logging and close file"""
        if not self.is_logging:
            return False
            
        try:
            self.is_logging = False
            if self.csv_file:
                self.csv_file.close()
                self.csv_file = None
                self.writer = None
            return True
        except IOError as e:
            print(f"Error stopping log: {e}")
            return False
    
    def export_data(self, data_buffer: DataBuffer, filename: str = None) -> bool:
        """Export all current buffer data to CSV"""
        try:
            if filename is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{Config.CSV_LOG_PREFIX}_export_{timestamp}.csv"
                
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    'timestamp', 'LDR1', 'LDR2', 'ServoAngle',
                    'V_fixed', 'I_fixed', 'P_fixed', 
                    'V_rot', 'I_rot', 'P_rot', 'PowerGain'
                ])
                
                for data in data_buffer.get_all_data():
                    writer.writerow([
                        data.timestamp, data.LDR1, data.LDR2, data.ServoAngle,
                        data.V_fixed, data.I_fixed, data.P_fixed,
                        data.V_rot, data.I_rot, data.P_rot, data.power_gain
                    ])
                    
            return True
        except IOError as e:
            print(f"Export error: {e}")
            return False


# =============================================================================
# VISUALIZATION COMPONENTS
# =============================================================================

class PowerGaugeWidget(QWidget):
    """Custom widget for displaying power gain as a gauge"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 120)
        self.gain_value = 0.0
        self.average_gain = 0.0
        
    def set_gain(self, current_gain: float, average_gain: float):
        """Update gauge values"""
        self.gain_value = current_gain
        self.average_gain = average_gain
        self.update()
        
    def paintEvent(self, event):
        """Custom paint event for gauge"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Setup colors
        bg_color = QColor(Config.COLORS['background'])
        text_color = QColor(Config.COLORS['foreground'])
        
        # Fill background
        painter.fillRect(self.rect(), bg_color)
        
        # Draw gauge background
        gauge_rect = self.rect().adjusted(10, 10, -10, -40)
        painter.setPen(QColor(100, 100, 100))
        painter.setBrush(QColor(50, 50, 50))
        painter.drawRect(gauge_rect)
        
        # Calculate gain color (green for positive, red for negative)
        gain_color = (QColor(Config.COLORS['gain_positive']) 
                     if self.average_gain >= 0 
                     else QColor(Config.COLORS['gain_negative']))
        
        # Draw gain bar
        bar_width = int(gauge_rect.width() * abs(self.average_gain) / 100.0)
        bar_width = min(bar_width, gauge_rect.width())
        
        if self.average_gain >= 0:
            bar_rect = gauge_rect.adjusted(0, 0, bar_width - gauge_rect.width(), 0)
        else:
            bar_rect = gauge_rect.adjusted(gauge_rect.width() - bar_width, 0, 0, 0)
            
        painter.fillRect(bar_rect, gain_color)
        
        # Draw center line
        center_x = gauge_rect.center().x()
        painter.setPen(QColor(Config.COLORS['foreground']))
        painter.drawLine(center_x, gauge_rect.top(), center_x, gauge_rect.bottom())
        
        # Draw text
        painter.setPen(text_color)
        painter.drawText(gauge_rect, Qt.AlignCenter, f"{self.average_gain:+.1f}%")
        
        # Draw label
        label_rect = self.rect().adjusted(10, self.rect().height() - 25, -10, -10)
        painter.drawText(label_rect, Qt.AlignCenter, "Power Gain")


class ServoVisualizationWidget(QWidget):
    """Widget for visualizing servo and panel orientation"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(200, 200)
        self.servo_angle = 90  # Default center position
        self.ldr1_value = 500
        self.ldr2_value = 500
        
    def update_data(self, servo_angle: float, ldr1: float, ldr2: float):
        """Update visualization data"""
        self.servo_angle = servo_angle
        self.ldr1_value = ldr1
        self.ldr2_value = ldr2
        self.update()
        
    def paintEvent(self, event):
        """Custom paint event for servo visualization"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Setup colors
        bg_color = QColor(Config.COLORS['background'])
        panel_color = QColor(Config.COLORS['accent'])
        base_color = QColor(100, 100, 100)
        ldr1_color = QColor(255, 200, 0, int(self.ldr1_value / 1024 * 200))
        ldr2_color = QColor(255, 200, 0, int(self.ldr2_value / 1024 * 200))
        
        # Fill background
        painter.fillRect(self.rect(), bg_color)
        
        center = self.rect().center()
        radius = min(self.rect().width(), self.rect().height()) // 3
        
        # Draw LDR indicators
        painter.setPen(Qt.NoPen)
        painter.setBrush(ldr1_color)
        painter.drawPie(center.x() - radius, center.y() - radius, 
                       radius * 2, radius * 2, 45 * 16, 90 * 16)
        
        painter.setBrush(ldr2_color)
        painter.drawPie(center.x() - radius, center.y() - radius, 
                       radius * 2, radius * 2, 225 * 16, 90 * 16)
        
        # Draw base
        painter.setPen(QPen(base_color, 3))
        painter.setBrush(base_color)
        base_rect = QRect(center.x() - 20, center.y() - 10, 40, 20)
        painter.drawRect(base_rect)
        
        # Draw rotating panel
        angle_rad = math.radians(self.servo_angle)
        panel_length = radius - 10
        
        end_x = center.x() + panel_length * math.cos(angle_rad)
        end_y = center.y() - panel_length * math.sin(angle_rad)
        
        painter.setPen(QPen(panel_color, 4))
        painter.drawLine(center.x(), center.y(), int(end_x), int(end_y))

        
        # Draw angle text
        painter.setPen(QColor(Config.COLORS['foreground']))
        painter.drawText(self.rect().adjusted(5, 5, -5, -5), 
                        Qt.AlignTop | Qt.AlignLeft, 
                        f"Angle: {self.servo_angle:.0f}°")


class PlotManager:
    """Manages pyqtgraph plots"""
    
    def __init__(self, plot_widget):
        self.plot_widget = plot_widget
        self.setup_plot()
        
        # Data arrays
        self.timestamps = []
        self.p_fixed_data = []
        self.p_rot_data = []
        
        # Plot curves
        self.curve_fixed = None
        self.curve_rot = None
        
    def setup_plot(self):
        """Configure the plot appearance"""
        self.plot_widget.setBackground(Config.COLORS['background'])
        self.plot_widget.setLabel('left', 'Power', 'mW')
        self.plot_widget.setLabel('bottom', 'Time', 's')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.addLegend()
        
        # Set dark theme colors
        axis_color = Config.COLORS['foreground']
        self.plot_widget.getAxis('left').setPen(axis_color)
        self.plot_widget.getAxis('bottom').setPen(axis_color)
        self.plot_widget.getAxis('left').setTextPen(axis_color)
        self.plot_widget.getAxis('bottom').setTextPen(axis_color)
        
    def update_plot(self, data_buffer: DataBuffer):
        """Update plot with latest data"""
        if data_buffer.count == 0:
            return
            
        data_points = data_buffer.get_all_data()
        current_time = time.time()
        
        # Extract data for plotting
        self.timestamps = [current_time - point.timestamp for point in data_points]
        self.p_fixed_data = [point.P_fixed for point in data_points]
        self.p_rot_data = [point.P_rot for point in data_points]
        
        # Clear previous curves
        self.plot_widget.clear()
        
        # Plot fixed panel data
        self.curve_fixed = self.plot_widget.plot(
            self.timestamps, self.p_fixed_data,
            pen=pg.mkPen(color=Config.COLORS['fixed_panel'], width=2),
            name='Fixed Panel'
        )
        
        # Plot rotating panel data
        self.curve_rot = self.plot_widget.plot(
            self.timestamps, self.p_rot_data,
            pen=pg.mkPen(color=Config.COLORS['rotating_panel'], width=2),
            name='Rotating Panel'
        )
        
        # Auto-scale to show recent data
        if self.timestamps:
            self.plot_widget.setXRange(max(self.timestamps[-1], Config.PLOT_TIME_WINDOW) - Config.PLOT_TIME_WINDOW, 
                                      max(self.timestamps[-1], Config.PLOT_TIME_WINDOW))


# =============================================================================
# MAIN DASHBOARD
# =============================================================================

class SolarDashboard(QMainWindow):
    """Main dashboard application window"""
    
    def __init__(self):
        super().__init__()
        
        # Initialize components
        self.data_buffer = DataBuffer()
        self.data_logger = DataLogger()
        self.serial_manager = SerialManager()
        self.data_simulator = DataSimulator()
        
        # UI state
        self.is_simulation_mode = False
        self.current_status = "Not Connected"
        self.gain_history = deque(maxlen=10)  # For rolling average
        
        # Initialize UI
        self.init_ui()
        self.init_connections()
        
        # Start timers
        self.setup_timers()
        
        # Auto-detect ports
        self.refresh_ports()
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("Solar Tracker Power Comparison Dashboard")
        self.setGeometry(100, 100, 1200, 800)
        
        # Apply dark theme
        self.apply_dark_theme()
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Connection panel
        main_layout.addWidget(self.create_connection_panel())
        
        # Status panel
        main_layout.addWidget(self.create_status_panel())
        
        # Visualization panel
        main_layout.addWidget(self.create_visualization_panel())
        
        # Sensors panel
        main_layout.addWidget(self.create_sensors_panel())
        
        # Logging panel
        main_layout.addWidget(self.create_logging_panel())
        
        # Status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        self.status_bar.showMessage("Ready - Select COM port and connect")
        
    def create_connection_panel(self) -> QWidget:
        """Create connection control panel"""
        panel = QGroupBox("Connection Settings")
        layout = QHBoxLayout(panel)
        
        # COM port selection
        layout.addWidget(QLabel("COM Port:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        layout.addWidget(self.port_combo)
        
        # Refresh button
        self.refresh_btn = QPushButton("Refresh")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        layout.addWidget(self.refresh_btn)
        
        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        layout.addWidget(self.connect_btn)
        
        # Status indicator
        self.status_indicator = QLabel("🔴 Not Connected")
        layout.addWidget(self.status_indicator)
        
        layout.addStretch()
        
        return panel
    
    def create_status_panel(self) -> QWidget:
        """Create system status panel"""
        panel = QGroupBox("System Status")
        layout = QHBoxLayout(panel)
        
        # Mode indicator
        self.mode_label = QLabel("Mode: Simulation")
        layout.addWidget(self.mode_label)
        
        # Power gain
        self.gain_label = QLabel("Gain: -- %")
        layout.addWidget(self.gain_label)
        
        # Data points
        self.data_count_label = QLabel("Data Points: 0")
        layout.addWidget(self.data_count_label)
        
        layout.addStretch()
        
        return panel
    
    def create_visualization_panel(self) -> QWidget:
        """Create main visualization panel"""
        panel = QWidget()
        layout = QHBoxLayout(panel)
        
        # Power comparison plot
        plot_group = QGroupBox("Power Comparison")
        plot_layout = QVBoxLayout(plot_group)
        
        if PYQTGRAPH_AVAILABLE:
            self.plot_widget = pg.PlotWidget()
            self.plot_manager = PlotManager(self.plot_widget)
            plot_layout.addWidget(self.plot_widget)
        else:
            plot_layout.addWidget(QLabel("pyqtgraph not available - install for plotting"))
        
        # Servo visualization
        servo_group = QGroupBox("Servo & Panel Orientation")
        servo_layout = QVBoxLayout(servo_group)
        self.servo_viz = ServoVisualizationWidget()
        servo_layout.addWidget(self.servo_viz)
        
        # Power gauge
        gauge_group = QGroupBox("Power Gain")
        gauge_layout = QVBoxLayout(gauge_group)
        self.power_gauge = PowerGaugeWidget()
        gauge_layout.addWidget(self.power_gauge)
        
        # Right side layout (servo + gauge)
        right_layout = QVBoxLayout()
        right_layout.addWidget(servo_group)
        right_layout.addWidget(gauge_group)
        
        layout.addWidget(plot_group, 2)  # 2/3 of width
        layout.addLayout(right_layout, 1)  # 1/3 of width
        
        return panel
    
    def create_sensors_panel(self) -> QWidget:
        """Create sensor visualization panel"""
        panel = QGroupBox("Light Sensors")
        layout = QVBoxLayout(panel)
        
        # LDR1
        ldr1_layout = QHBoxLayout()
        ldr1_layout.addWidget(QLabel("LDR1:"))
        self.ldr1_bar = QProgressBar()
        self.ldr1_bar.setRange(0, 1024)
        self.ldr1_bar.setFormat("%v")
        ldr1_layout.addWidget(self.ldr1_bar)
        self.ldr1_value = QLabel("0")
        ldr1_layout.addWidget(self.ldr1_value)
        layout.addLayout(ldr1_layout)
        
        # LDR2
        ldr2_layout = QHBoxLayout()
        ldr2_layout.addWidget(QLabel("LDR2:"))
        self.ldr2_bar = QProgressBar()
        self.ldr2_bar.setRange(0, 1024)
        self.ldr2_bar.setFormat("%v")
        ldr2_layout.addWidget(self.ldr2_bar)
        self.ldr2_value = QLabel("0")
        ldr2_layout.addWidget(self.ldr2_value)
        layout.addLayout(ldr2_layout)
        
        # Difference indicator
        self.diff_label = QLabel("Difference: --")
        layout.addWidget(self.diff_label)
        
        return panel
    
    def create_logging_panel(self) -> QWidget:
        """Create data logging controls"""
        panel = QWidget()
        layout = QHBoxLayout(panel)
        
        # Logging controls
        self.start_log_btn = QPushButton("Start Logging")
        self.start_log_btn.clicked.connect(self.start_logging)
        layout.addWidget(self.start_log_btn)
        
        self.stop_log_btn = QPushButton("Stop Logging")
        self.stop_log_btn.clicked.connect(self.stop_logging)
        self.stop_log_btn.setEnabled(False)
        layout.addWidget(self.stop_log_btn)
        
        self.export_btn = QPushButton("Export CSV")
        self.export_btn.clicked.connect(self.export_data)
        layout.addWidget(self.export_btn)
        
        # Logging status
        self.log_status_label = QLabel("Not logging")
        layout.addWidget(self.log_status_label)
        
        layout.addStretch()
        
        return panel
    
    def apply_dark_theme(self):
        """Apply dark theme to the application"""
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {Config.COLORS['background']};
                color: {Config.COLORS['foreground']};
            }}
            QGroupBox {{
                font-weight: bold;
                border: 1px solid {Config.COLORS['accent']};
                border-radius: 5px;
                margin-top: 1ex;
                padding-top: 1ex;
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: {Config.COLORS['accent']};
            }}
            QPushButton {{
                background-color: #404040;
                color: {Config.COLORS['foreground']};
                border: 1px solid #606060;
                padding: 5px 10px;
                border-radius: 3px;
            }}
            QPushButton:hover {{
                background-color: #505050;
            }}
            QPushButton:pressed {{
                background-color: #303030;
            }}
            QComboBox {{
                background-color: #404040;
                color: {Config.COLORS['foreground']};
                border: 1px solid #606060;
                padding: 5px;
                border-radius: 3px;
            }}
            QProgressBar {{
                border: 1px solid #606060;
                border-radius: 3px;
                text-align: center;
                color: {Config.COLORS['foreground']};
            }}
            QProgressBar::chunk {{
                background-color: {Config.COLORS['accent']};
                border-radius: 2px;
            }}
        """)
    
    def init_connections(self):
        """Initialize signal connections"""
        # Serial manager signals
        self.serial_manager.data_received.connect(self.handle_new_data)
        self.serial_manager.status_update.connect(self.handle_status_update)
        self.serial_manager.connection_changed.connect(self.handle_connection_changed)
    
    def setup_timers(self):
        """Setup QTimers for periodic tasks"""
        # GUI update timer
        self.gui_timer = QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(Config.UPDATE_INTERVAL_MS)
        
        # Simulation timer (if no Arduino)
        self.simulation_timer = QTimer()
        self.simulation_timer.timeout.connect(self.generate_simulation_data)
        
        # Auto-reconnect timer
        self.reconnect_timer = QTimer()
        self.reconnect_timer.timeout.connect(self.attempt_reconnect)
    
    def refresh_ports(self):
        """Refresh available COM ports"""
        self.port_combo.clear()
        
        if not SERIAL_AVAILABLE:
            self.port_combo.addItem("pySerial not available")
            return
            
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device, port.device)
        
        if self.port_combo.count() == 0:
            self.port_combo.addItem("No ports available")
    
    def toggle_connection(self):
        """Toggle serial connection"""
        if self.serial_manager.is_connected:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """Connect to selected serial port"""
        port = self.port_combo.currentData()
        if port and port != "No ports available":
            self.serial_manager.connect_to_port(port)
            self.serial_manager.start()
        else:
            self.enter_simulation_mode()
    
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.serial_manager.disconnect()
        self.enter_simulation_mode()
    
    def enter_simulation_mode(self):
        """Enter simulation mode"""
        self.is_simulation_mode = True
        self.simulation_timer.start(1000 // Config.PLOT_UPDATE_RATE)
        self.mode_label.setText("Mode: Simulation")
        self.status_indicator.setText("🟡 Simulation Mode")
        self.status_bar.showMessage("Running in simulation mode - no Arduino connected")
    
    def exit_simulation_mode(self):
        """Exit simulation mode"""
        self.is_simulation_mode = False
        self.simulation_timer.stop()
    
    def attempt_reconnect(self):
        """Attempt to reconnect to Arduino"""
        if not self.serial_manager.is_connected and not self.is_simulation_mode:
            self.refresh_ports()
            if self.port_combo.count() > 0 and self.port_combo.currentData() != "No ports available":
                self.connect_serial()
    
    def handle_new_data(self, data: SolarData):
        """Handle new data from serial manager"""
        if self.data_buffer.add_data(data):
            # Log data if logging is active
            if self.data_logger.is_logging:
                self.data_logger.log_data(data)
            
            # Update gain history for rolling average
            self.gain_history.append(data.power_gain)
    
    def handle_status_update(self, status_type: str, message: str):
        """Handle status updates from serial manager"""
        color_map = {
            'success': 'green',
            'error': 'red', 
            'warning': 'orange',
            'info': 'blue'
        }
        
        color = color_map.get(status_type, 'black')
        self.status_bar.showMessage(message)
        print(f"[{status_type.upper()}] {message}")
    
    def handle_connection_changed(self, connected: bool):
        """Handle connection state changes"""
        if connected:
            self.exit_simulation_mode()
            self.status_indicator.setText("✅ Connected")
            self.connect_btn.setText("Disconnect")
            self.current_status = "Connected"
            self.reconnect_timer.stop()
        else:
            self.status_indicator.setText("🔴 Not Connected") 
            self.connect_btn.setText("Connect")
            self.current_status = "Not Connected"
            
            # Only start reconnect timer if we're not in manual simulation mode
            if not self.is_simulation_mode:
                self.reconnect_timer.start(Config.AUTO_RECONNECT_INTERVAL_MS)
    
    def generate_simulation_data(self):
        """Generate simulated data for testing"""
        simulated_data = self.data_simulator.generate_sample()
        self.handle_new_data(simulated_data)
    
    def update_gui(self):
        """Update all GUI elements"""
        # Update data count
        self.data_count_label.setText(f"Data Points: {self.data_buffer.count}")
        
        # Get latest data
        latest_data = self.data_buffer.last_valid_data
        if latest_data:
            # Update power gain display
            current_gain = latest_data.power_gain
            avg_gain = sum(self.gain_history) / len(self.gain_history) if self.gain_history else 0
            
            gain_color = Config.COLORS['gain_positive'] if avg_gain >= 0 else Config.COLORS['gain_negative']
            self.gain_label.setText(f'Gain: <span style="color: {gain_color}">{avg_gain:+.1f}%</span>')
            
            # Update power gauge
            self.power_gauge.set_gain(current_gain, avg_gain)
            
            # Update servo visualization
            self.servo_viz.update_data(
                latest_data.ServoAngle,
                latest_data.LDR1,
                latest_data.LDR2
            )
            
            # Update sensor displays
            self.ldr1_bar.setValue(int(latest_data.LDR1))
            self.ldr1_value.setText(f"{latest_data.LDR1:.0f}")
            self.ldr2_bar.setValue(int(latest_data.LDR2))
            self.ldr2_value.setText(f"{latest_data.LDR2:.0f}")
            
            # Update difference
            diff = latest_data.LDR1 - latest_data.LDR2
            diff_text = f"Difference: {diff:+.0f}"
            self.diff_label.setText(diff_text)
        
        # Update plot
        if PYQTGRAPH_AVAILABLE:
            self.plot_manager.update_plot(self.data_buffer)
        
        # Update logging status
        log_status = "Logging" if self.data_logger.is_logging else "Not logging"
        self.log_status_label.setText(log_status)
    
    def start_logging(self):
        """Start data logging"""
        if self.data_logger.start_logging():
            self.start_log_btn.setEnabled(False)
            self.stop_log_btn.setEnabled(True)
            self.status_bar.showMessage("Started logging data")
        else:
            self.status_bar.showMessage("Failed to start logging")
    
    def stop_logging(self):
        """Stop data logging"""
        if self.data_logger.stop_logging():
            self.start_log_btn.setEnabled(True)
            self.stop_log_btn.setEnabled(False)
            self.status_bar.showMessage("Stopped logging data")
        else:
            self.status_bar.showMessage("Failed to stop logging")
    
    def export_data(self):
        """Export all data to CSV"""
        filename, _ = QFileDialog.getSaveFileName(
            self, "Export CSV", 
            f"{Config.CSV_LOG_PREFIX}_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            "CSV Files (*.csv)"
        )
        
        if filename:
            if self.data_logger.export_data(self.data_buffer, filename):
                self.status_bar.showMessage(f"Data exported to {filename}")
                QMessageBox.information(self, "Export Successful", f"Data exported to {filename}")
            else:
                self.status_bar.showMessage("Export failed")
                QMessageBox.warning(self, "Export Failed", "Could not export data")
    
    def closeEvent(self, event):
        """Handle application close"""
        # Stop serial communication
        self.serial_manager.stop()
        
        # Stop logging
        if self.data_logger.is_logging:
            self.data_logger.stop_logging()
        
        event.accept()


# =============================================================================
# APPLICATION ENTRY POINT
# =============================================================================

def main():
    """Main application entry point"""
    
    # Check for required dependencies
    if not PYQT5_AVAILABLE:
        print("Error: PyQt5 is required but not installed.")
        print("Install it with: pip install PyQt5")
        return 1
        
    if not PYQTGRAPH_AVAILABLE:
        print("Warning: pyqtgraph not installed - plots will not work")
        print("Install it with: pip install pyqtgraph")
    
    if not SERIAL_AVAILABLE:
        print("Warning: pySerial not installed - serial communication disabled")
        print("Install it with: pip install pyserial")
    
    # Import math here to avoid issues if dependencies are missing
    import math
    
    # Create application
    app = QApplication(sys.argv)
    app.setApplicationName("Solar Tracker Dashboard")
    app.setApplicationVersion("1.0")
    
    # Create and show main window
    window = SolarDashboard()
    window.show()
    
    # Start application
    try:
        return app.exec_()
    except KeyboardInterrupt:
        print("Application interrupted by user")
        return 0


if __name__ == "__main__":
    sys.exit(main())