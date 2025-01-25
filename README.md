# IoT Project 2024

## Project Overview
This project focuses on simulating and optimizing the movement and data handling of IoT sensors, balloons and base station in a defined simulation area. The simulation achieves efficient data collection, caching, and communication while balancing memory constraints and performance requirements.

### Key Features
1. **Active Sensor Movement**:
   - Sensors move along the X-axis with randomized directions.
   - Collisions and overlapping are avoided through Y-coordinate management and spawn orientation control.

2. **Balloon Deployment**:
   - Balloons are deployed symmetrically along the X-axis to ensure coverage of mobile sensors.
   - Positions are calculated dynamically based on the simulation area and the number of balloons.

3. **Dynamic Caching**:
   - Implemented an LRU (Least Recently Used) cache replacement policy.
   - Balloons handle data requests efficiently by reordering the cache based on recent activity and removing expired entries dynamically.

4. **Request Distribution**:
   - Base station requests are sent using Poisson, Gaussian, and pseudo-random distributions for realistic polling of sensor data.
   - The request generation parameters are dynamically adjusted for optimal performance.

5. **Performance Optimization**:
   - Cache size and polling frequency are tuned to balance memory usage and request satisfaction.
   - Detailed statistics and performance metrics are displayed during the simulation.

6. **GUI Integration**:
   - Used `dearpygui` to visualize balloon caches and simulation statistics dynamically.

---

## File Structure
- **`simulation_launch`**: Initializes the simulation, spawning sensors and balloons.
- **`fleet_coordinator`**: Handles balloon positioning and distance calculations.
- **`simulation_manager`**: Manages data exchange between sensors, balloons, and the base station.
- **`base_station_controller`**: Manages base station requests and data handling.
- **`sensor_controller`**: Controls sensor movement and data generation.

---

## Performance Considerations
- Cache size directly impacts request satisfaction and cache misses.
- Dynamic adjustments to timestamp ranges and polling frequency allow for efficient simulations under varying conditions.
- Balloon stabilization phases at startup may cause some data loss, simulating realistic adjustment scenarios.

---

## Additional Notes
- Key implementation details are commented in the shared code and in the following reports

## [Text Report](https://github.com/FedBio01/IoT-project/blob/6e05a9f91bf7f9994e06323dbd01461feca1e17b/Report_IoT_2024_Biondi-Casciani-Di%20Paola.pdf)
## [Detailed Simulations Report](https://github.com/FedBio01/IoT-project/blob/main/IoT_Project_C_Simulations_2024.pdf)

## [Video link](https://youtu.be/uOjWqldErBI)

# Members:
- Biondi Federico 2151856, biondi.2151856@studenti.uniroma1.it
- Casciani Leonardo 2154695, casciani.2154695@studenti.uniroma1.it
- Di Paola Riccardo 2151847, dipaola.2151847@studenti.uniroma1.it


