
I would first start by having a concrete understanding of ECAL threading with pub/sub and services.

# One Module to Subscribe to ECAL PUB SUB Messages
    * Subscribe to RGBD Images(30 HZ) and Pose (200/100 HZ)
    * Check state variables to perform scans or integrate. Note that the RGBD camera already has 6DOF pose so no sync is needed between pose subscription
    * Publish POSE on serial? (mutex lock) 
# One Module to have an RPC Server
    * Start/Stop Single Scans
        * Sets a GLOBAL (mutex controled) variable to integrate scans
        * RGBD -> OPC -> Mesh -> Polygons -> Landing Site -> Cirlce -> **Touchdown Point (Global and Body Frame)**
    * Start Volume Integration
        * Sets a GLOBAL (mutex controled) variable to start volume integration
        * Integrate RGBD Images -> Mesh -> Polygons -> Landing Site -> Cirlce -> **Touchdown Point (Global and Body Frame)**
    * Execute Landing
        * Using Last Touchdown Site, COmmand Drone (Serial) to land
        *  


# One Module to Handle Landing 