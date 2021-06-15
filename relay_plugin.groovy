// DRONE FUNCTIONS

def return_home = { drone ->
    if((String)drone.cmd == 'return') return
    if(drone.state == active) {
        println(drone.id + " is returning home")
        cancel drone
        run { home drone }
    }
}

def move_to = { drone, lat, lon, alt ->
    if(drone.distance([lat: lat, lon: lon])>50.cm) {
        if((String)drone.cmd == 'goto' && drone.cmd.target.distance([lat: lat, lon: lon])<50.cm) return
        if((String)drone.cmd == 'return') {
            cancel drone
        }
	println("state " + drone.id + ": " + (String)drone.state)
        if((String)drone.state == 'ready' || drone.state == ready) {
		println("passou pq tava ready")
		return
	}
	println(drone.id + " move to " + lat + ", " + lon)
        run { move drone to lat: lat, lon: lon, speed: 9 }
    }
}

def get_drone = {
    drone = assign any
    params.set drone, tags:['relay']
    arm drone
    relay_drones.add(drone)
    run { takeoff drone, 8.m}
    return drone
}

// AUXILIARY FUNCTIONS

public class Utils {

    static public calculateDistance = { position1, position2 ->
        // Convert the latitudes and longitudes from degree to radians.
        double lat1 = Math.toRadians(position1.lat)
        double lon1 = Math.toRadians(position1.lon)
        double lat2 = Math.toRadians(position2.lat)
        double lon2 = Math.toRadians(position2.lon)

        // Haversine Formula
        double dlon = lon2 - lon1
        double dlat = lat2 - lat1

        double distance = (Math.sin(dlat / 2)**2) + Math.cos(lat1) * Math.cos(lat2) * (Math.sin(dlon / 2)**2)
        return 2 * Math.asin(Math.sqrt(distance)) * 6371000
    }

}

// SEARCH CLASSES

public class Node {
	def state = null;
	Node parent = null;
	int depth = null;
    float heuristic = 0;

	public Node(state, Node parent, int depth) {
		this.state = state;
        this.parent = parent;
        this.depth = depth;
	}

    def apply_heuristics = {
        this.heuristic = this.state.heuristic
    }

    def in_parent = { state ->
        if(this.state == state) return true
        if(this.parent == null) return false
        return self.parent.in_parent(state)
    }

    def hash = {
        return this.state.hash()
    }
}

public class DronesMovimentation {
    def movements = null;
    def mission_goals = null;
    def longest_path_length = null;
    def sum_path_length = 0;    

    public DronesMovimentation(movements, missing_goals) {
        this.movements = movements
        this.missing_goals = missing_goals
    }

    public heuristic = {
        return 1
    }

}

public class RelayLink {
    def entity = null;
    def entity_id = null;
    def coord_drone_side;
    double distance = 0;
    RelayLink child = null;
    double optimal_network = 65;
    double network_threshold = 78;
    double maximum_distance = 70;
    double minimum_distance = 10;
    // target_desire represents the desire for the drone to reach its target, if this value is smaller the drone will try to reach the target with caution, if its larger the drone will try to reach the target position faster with possible overshoot, this is used to improve the network
    double target_desire = 1/100;
    double prediction_time = 2;
    double acceptance_radius = 5;

    public RelayLink(def coord_drone_side, def child, def distance) {
        this.coord_drone_side = coord_drone_side
        this.distance = distance
        this.child = child
    }

    public update_entity = { entity ->
        this.entity = entity
        if(entity == null) this.entity_id = null
        else this.entity_id = entity.id
    }

    public size = {
        def aux = this
        def counter = 1
        while(aux.child != null) {
            counter = counter + 1
            aux = aux.child
        }
        return counter
    }

    public get = { i ->
        def aux = this
        def counter = 0
        while(True) {
            if(i == counter) return aux
            counter = counter + 1
            aux = aux.child
            if (aux.child == null) return null
        }
    }

    public str = {
        def RESET_COLOR = "\u001B[0m";
        def BLUE = "\u001B[34m";
        return BLUE + "relay link: {" + RESET_COLOR + "coord: [lat: " + coord_drone_side.lat + ", lon: " + coord_drone_side.lon + "]" + ", entity_id: [" + entity_id + "]" + ", entity: [" + entity + "]" + BLUE + "}" + RESET_COLOR
    }
    
}

public class DroneRelayBridge {
    def mission_drone;
    def relay_links = [];
    def connected_relay_link;

    public DroneRelayBridge(mission_drone, relay_link) {
        this.mission_drone = mission_drone
        double distance = Utils.calculateDistance(this.mission_drone.position, relay_link.entity.position)
        def new_link = new RelayLink(mission_drone.position, relay_link, distance)
        new_link.update_entity(mission_drone)
        this.relay_links.add(new_link)
        this.connected_relay_link = relay_link
    }

    public update_mission_drone = { mission_drone ->
        this.mission_drone = mission_drone
        this.relay_links[0].coord_drone_side = [lat: mission_drone.position.lat, lon: mission_drone.position.lon]
    }
    
    public update_relay_bridge = { closest_relay_link, networks ->
        this.connected_relay_link = closest_relay_link

        /*
        // predict future positions for the first relay link (mission drone)
        if(this.mission_drone.cmd != null && ['goto'].contains((String)this.mission_drone.cmd)) {
            // THIS IS NOT CORRECT, FOR SOME REASON THE DRONE.CMD.TARGET IS NOT PROVIDING THE CORRECT COORDINATE !!!!!
            def target = [lat: this.mission_drone.cmd.target.lat, lon: this.mission_drone.cmd.target.lon]
            println("target of " + this.mission_drone.id + ": " + target)
            if(this.relay_links[0].child != closest_relay_link) {
                println("position of " + this.relay_links[0].child.entity.id + ": " + this.relay_links[0].child.entity.position)
            }
            // calculate unitary vector to target position
            def vector_predicted = [lat: target.lat-this.mission_drone.position.lat, lon: target.lon-this.mission_drone.position.lon]

            double distance_to_target = Utils.calculateDistance(target, this.mission_drone.position)

            double lat_predicted = vector_predicted.lat/distance_to_target
            double lon_predicted = vector_predicted.lon/distance_to_target
            def unitary_vector_to_target = [lat: lat_predicted, lon: lon_predicted]

            // update drone side coord based on the prediction of the future position of the mission drone
            this.relay_links[0].coord_drone_side = [
                lat: this.mission_drone.position.lat + (unitary_vector_to_target.lat * this.relay_links[0].prediction_time * this.mission_drone.speed),
                lon: this.mission_drone.position.lon + (unitary_vector_to_target.lon * this.relay_links[0].prediction_time * this.mission_drone.speed)
            ]
            
            // in case of overshoot, fix to target coordinate
            if(Utils.calculateDistance(this.relay_links[0].entity.position, this.relay_links[0].coord_drone_side) > distance_to_target) {
                this.relay_links[0].coord_drone_side = target
            }
        }
        else {
            this.relay_links[0].coord_drone_side = [lat: this.mission_drone.position.lat, lon: this.mission_drone.position.lon]
        }
        */

        this.relay_links[0].coord_drone_side = [lat: this.mission_drone.position.lat, lon: this.mission_drone.position.lon]

        // calculate unitary vector from drone to connected coord
        def connected_coord = this.connected_relay_link.coord_drone_side
        def vector = [lat: connected_coord.lat-this.relay_links[0].coord_drone_side.lat, lon: connected_coord.lon-this.relay_links[0].coord_drone_side.lon]
        double distance = Utils.calculateDistance(this.mission_drone.position, connected_coord)
        double lat = vector.lat/distance
        double lon = vector.lon/distance
        def unitary_vector = [lat: lat, lon: lon]

        // network variables
        def network = null;
        def signal = null;
        def quality = null;

        def distance_counter = 0;

        // change bridge positions for better connection
        for(int i=0; i<relay_links.size()-1; i++) {
            def relay_link = this.relay_links[i]
            def distance_to_position = Utils.calculateDistance(relay_link.entity.position, relay_link.coord_drone_side)
            def child_distance_to_position = Utils.calculateDistance(relay_link.child.entity.position, relay_link.child.coord_drone_side)

            // retrieve information from network parameters
            network = networks[relay_link.entity.id];
            signal = 90;
            def signal_to_base = 90;
            for(con: network) {
                if(con["Relay"] == relay_links[i].child.entity.id) {
                    signal = Double.parseDouble(con["Signal"])
                    quality = con["NetworkQuality"]
                }
                if(con["Relay"] == connected_relay_link.entity.id) {
                    signal_to_base = Double.parseDouble(con["Signal"])
                }
            }

            distance_counter += this.relay_links[i].distance
            
            // update distance based on network parameters
            if(signal < 80) {
                double deltaX = -relay_link.target_desire*Math.pow(signal-relay_link.optimal_network, 3)
                if(Math.abs(deltaX) > 10) {
                    deltaX = 8 * Math.signum(deltaX)
                }
                
                def distance_to_child = Utils.calculateDistance(relay_link.entity.position, relay_link.child.entity.position)
                def new_distance = distance_to_child + deltaX
                if(new_distance < relay_link.minimum_distance) this.relay_links[i].distance = relay_link.minimum_distance
                else this.relay_links[i].distance = new_distance

                // remove excess drone relays
                if((child_distance_to_position < this.relay_links[i].acceptance_radius && signal_to_base < signal) || distance_counter > distance || signal_to_base < relay_link.optimal_network) {
                    this.relay_links[i].child = this.connected_relay_link
                    for(int j=i+1;j<this.relay_links.size();j++)
                        this.relay_links.remove(j)
                    return;
                }
            }
            
            // calculate new coordinates for network side drone
            def drone_side_coords = relay_link.coord_drone_side
            double lat_pos = drone_side_coords.lat + (unitary_vector.lat * this.relay_links[i].distance)
            double lon_pos = drone_side_coords.lon + (unitary_vector.lon * this.relay_links[i].distance)
            this.relay_links[i].child.coord_drone_side = [lat: lat_pos, lon: lon_pos]
        }

        if(!networks.containsKey(relay_links[-1].entity.id) || networks[relay_links[-1].entity.id]==null) return

        // get signal from the last relay link
        signal = null
        network = networks[relay_links[-1].entity.id]
        for(con: network) {
            //signal = Double.parseDouble(con["Signal"])
            if(con.Relay == connected_relay_link.entity.id) {
                signal = Double.parseDouble(con["Signal"])
            }
        }

        // add new drones in bridge if signal is worst than 75db or last drone is more than 80 meters away from connected drone
        if(signal != null && signal > this.relay_links[-1].network_threshold || Utils.calculateDistance(relay_links[-1].coord_drone_side, connected_relay_link.entity.position) > relay_links[-1].maximum_distance) {
            // calculate middle point
            def last_relay_link = this.relay_links[-1]
            double new_lat = (last_relay_link.coord_drone_side.lat + connected_coord.lat)/2
            double new_lon = (last_relay_link.coord_drone_side.lon + connected_coord.lon)/2
            def middle_point = [lat: new_lat, lon: new_lon]

            // create the last relay link
            double half_distance = Utils.calculateDistance(last_relay_link.entity.position, connected_relay_link.entity.position)/2
            def new_relay_link = new RelayLink(middle_point, this.connected_relay_link, half_distance)

            // update second to last relay link
            this.relay_links[-1].child = new_relay_link
            this.relay_links[-1].distance = half_distance

            this.relay_links.add(new_relay_link)
        }
    }

    public str = {
        def RESET_COLOR = "\u001B[0m";
        def GREEN = "\u001B[32m";
        def s = GREEN + "relay bridge: {" + RESET_COLOR
        def count = 0;
        for(relay_link: this.relay_links) {
            s = s+relay_link.str()
            if(count < relay_links.size())
                s = s+", "
            count++;
        }
        return s + this.connected_relay_link.str() + GREEN + "}" + RESET_COLOR
    }

}

public class NetworkState {
    def relay_bridges = [:];            // drone.id : DroneRelayBridge
    def mission_drones = [];            // Drone[]
    def groundstations = [];            // Groundstation[]
    def connected_links = []//.toSet();   // RelayLink[]
    def networks = [:]                  // drone.id : SensorInfo
    def bridges_stack = [];             // DroneRelayBridge[]

    public NetworkState(groundstations) {
        this.groundstations = groundstations
        for(gs: groundstations) {
            def new_relay_link = new RelayLink(gs.position, null, 0)
            new_relay_link.update_entity(gs)
            connected_links.add(new_relay_link)
        }
    }

    public update_mission_drones = { mission_drones ->
        this.mission_drones = mission_drones
        for(drone: mission_drones)
            if(this.relay_bridges.containsKey(drone.id))
                this.relay_bridges[drone.id].update_mission_drone(drone)
    }

    public add_new_relay_bridges = { mission_drones ->
        for(drone: mission_drones) {
            if(!this.relay_bridges.containsKey(drone.id)) {
                def new_relay_bridge = new DroneRelayBridge(drone, this.connected_links[0])
                this.relay_bridges[drone.id] = new_relay_bridge
            }
        }
    }

    public get_relay_bridges = {
        def array = []
        for(id: this.relay_bridges.keySet() as List)
            array.add(this.relay_bridges[id])
        return array
    }

    public has = { entity ->
        for(relay_bridge: this.get_relay_bridges()) {
            for(relay_link: relay_bridge.relay_links) {
                if(entity == relay_link.entity) {
                    return true
                }
            }
        }
        return false
    }

    public update_relay_bridges = {
        // deepcopy connected links
        def connected_relay_links = []
        for(int i=0;i<this.connected_links.size(); i++) {
            connected_relay_links.add(this.connected_links[i])
        }

        def mission_drones_to_update = []

        // populate mission_drones_to_update
        for(drone: this.mission_drones)
            mission_drones_to_update.add(drone)

        while(mission_drones_to_update.size()>0) {
            // get closest drone to connected drones
            def aux = get_closest_mission_drone_to_connected(connected_relay_links, mission_drones_to_update)
            def closest_drone = aux.drone
            def closest_relay_link = aux.relay_link

            // update relay bridge of closest drone
            this.relay_bridges[closest_drone.id].update_relay_bridge(closest_relay_link, networks)

            // add relay bridge to the stack
            bridges_stack.add(0, this.relay_bridges[closest_drone.id])

            // update connected coords
            for(int i=0;i<this.relay_bridges[closest_drone.id].relay_links.size(); i++)
                connected_relay_links.add(this.relay_bridges[closest_drone.id].relay_links[i])

            // remove drone
            mission_drones_to_update.remove(closest_drone)
        }
    }

    def get_closest_relay_drone_returning_base = { relay_drones, position ->
        def closest_distance = null;
        def closest_drone = null;
        double distance;
        for(relay_drone: relay_drones) {
            if(!this.has(relay_drone)) {
                distance = Utils.calculateDistance(position, relay_drone.position)
                if(closest_distance == null || distance < closest_distance) {
                    closest_distance = distance
                    closest_drone = relay_drone
                }
            }
        }
        return [drone: closest_drone, distance: closest_distance]
    }

    def get_closest_mission_drone_to_connected = { connected_relay_links, remaining_drones ->
        def closest_distance = null;
        def closest_drone = null;
        RelayLink closest_relay_link = null;
        double distance;
        for(relay_link: connected_relay_links) {
            for(drone: remaining_drones) {
                distance = Utils.calculateDistance(drone.position, relay_link.coord_drone_side)
                if(closest_distance == null || distance < closest_distance) {
                    closest_distance = distance
                    closest_drone = drone
                    closest_relay_link = relay_link
                }
            }
        }
        return [drone: closest_drone, relay_link: closest_relay_link]
    }

    public str = {
        def RESET_COLOR = "\u001B[0m";
        def PURPLE = "\u001B[35m";
        def s = PURPLE + "network state: {" + RESET_COLOR
        def count = 0;
        for(relay_bridge: this.get_relay_bridges())
            s = s+relay_bridge.str() + ", "
        return s + PURPLE + "}" + RESET_COLOR
    }
}

// INIT FUNCTION

def init_state = {
    relay_drones = []
    groundstations = []

    if(simulation) {
        groundstation.params = [
            ipAddress: groundstation.containsKey('ip') ? groundstation.ip: '0.0.0.0',
            macAddress:groundstation.containsKey('mac') ? groundstation.mac: '0.0.0.0',
            tags: []
        ]
    }
    else {
        groundstation.params = [
            ipAddress: groundstation.ip,
            macAddress: groundstation.mac,
            tags: []
        ]
    }
    

    groundstations.add(groundstation)

    network_state = new NetworkState(groundstations)
}

// RELAY ALGORITHM

def relay = { drones ->
    if(drones == [] || drones == null) return

    def mission_drones = []
    relay_drones = []

    // refresh sensors    
    for(relay_bridge: network_state.get_relay_bridges()) {
        for(relay_link: relay_bridge.relay_links) {
            if(relay_link.child != null)  {
                def drone_id = relay_link.entity.id
                def network = sensor.(drone_id).network
                println("network: " + network)
		if(network != null) {
                    if(simulation) {
                        network = network.simulated
                    }
                    else {
                        network = network.real
                    }
			//println(network)
                    def value = network[0]["groundStation"]

                    //println(drone_id + " sensor: " + value)
                    network_state.networks[drone_id] = value
                }
            }
        }
    }

    // get mission drones from all drones
    for(drone in drones) {
        if(drone.params.tags.contains('relay')) {
            relay_drones.add(drone)
        }
        else {
            mission_drones.add(drone)
        }
    }

    // add new relay bridge for new mission_drones
    network_state.update_mission_drones(mission_drones)
    network_state.add_new_relay_bridges(mission_drones)

    // recalculate bridges for moving drones
    network_state.update_relay_bridges()

    // update connected entities
    def connected_entities = [].toSet()
    for(link: network_state.connected_links) connected_entities.add(link.entity)

    // reposition drones with space constraints
    while(network_state.bridges_stack.size() > 0) {
        // get bridges from the leafs to the core
        def bridge = network_state.bridges_stack.pop()
        for(relay_link: bridge.relay_links) {
            // if the child isnt null, it means the child has already a drone assigned to that position
            if(relay_link.entity != null) continue
            def temp_relay_link = relay_link.child

            // get closest drone relay returning to base
            def returning_base_relay_drone = network_state.get_closest_relay_drone_returning_base(relay_drones, relay_link.coord_drone_side)

            // go through the relay links until reaching the links which have connection
            while(!connected_entities.contains(temp_relay_link.entity)) {
                
                // if the child relay link has a relay drone ready to be used
                if(temp_relay_link.entity != null && relay_drones.contains(temp_relay_link.entity)) {
                    
                    // place returning drone relay
                    if(returning_base_relay_drone.drone != null) {
                        double distance_from_relay_drone_to_final_position = Utils.calculateDistance(temp_relay_link.entity.position, relay_link.coord_drone_side)

                        // and relay drone returning is closer that the child
                        if(returning_base_relay_drone.distance < distance_from_relay_drone_to_final_position) {
                            // add returning drone relay to the network
                            relay_link.update_entity(returning_base_relay_drone.drone)
                            break
                        }
                    }
                    // switch drones positions with child
                    relay_link.update_entity(temp_relay_link.entity)
                    temp_relay_link.update_entity(null)
                    break
                }
                temp_relay_link = temp_relay_link.child
            }
        }
    }
	println(network_state.str())
    // send drones to respective positions
    for(relay_bridge: network_state.get_relay_bridges()) {
        for(int i=1; i<relay_bridge.relay_links.size(); i++) {

            // when drones havent been dispached yet
            if(relay_bridge.relay_links[i].entity == null) {
                def drone;
                // assign new drone or getting the closest returning relay drones -> to improve: check of its worth it to just return this drone to base and bring another one. ex: one drone on top needs relay and one drone in the bottom of the map is available but there are other drones already available in the groundstation which is in the middle.
                def returning_base_relay_drone = network_state.get_closest_relay_drone_returning_base(relay_drones, relay_bridge.relay_links[i].coord_drone_side)
                if(returning_base_relay_drone.drone == null) {
                    drone = get_drone()
                }
                else {
                    drone = returning_base_relay_drone.drone
                }
                relay_bridge.relay_links[i].update_entity(drone)
                def coords = relay_bridge.relay_links[i].coord_drone_side
                move_to(drone, coords.lat, coords.lon, 5)
            }
            // move relay drone
            else if(relay_drones.contains(relay_bridge.relay_links[i].entity)) {
                def coords = relay_bridge.relay_links[i].coord_drone_side
                def drone_id = relay_bridge.relay_links[i].entity_id
                for(drone: relay_drones) {
                    if(drone.id == drone_id) {
                        move_to(drone, coords.lat, coords.lon, 5)
                    }
                }
            }
        }
    }

    // actually remove drones in excess
    for(drone: relay_drones) {
        if(!network_state.has(drone) ) {
            return_home(drone)
        }
    }

    // request refresh of sensor information
    for(relay_bridge: network_state.get_relay_bridges()) {
        for(relay_link: relay_bridge.relay_links) {
            if(relay_link.child != null)  {
                def entity_ids = []
                def entity_ips = []
                def entity_macs = []
                for(entity: drones + groundstations) {
                    if(entity.id != relay_link.entity.id) {
                        entity_ids.add(entity.id)
                        entity_ips.add(entity.params.ipAddress)
                        entity_macs.add(entity.params.macAddress)
                    }
                }
                send droneId: relay_link.entity.id, connectionsId: entity_ids, connectionsIp: entity_ips, connectionsMac: entity_macs
            }
        }
    }

    //println(network_state.str())
    println("##################################################################################")

}

plugin {
    id      'new_relay'
    type    scheduled
    input   rate: Time,
            groundstation: [Map, [
                id: "groundStation",
                position: [lat: 40.633874, lon: -8.660311],
                ip: '0.0.0.0',
                mac: '0.0.0.0']
            ],
            simulation: [Boolean, false]
    vars    network_state: [:],
            groundstations: [:],
            relay_drones: []
    callback relay
    init    init_state
}
