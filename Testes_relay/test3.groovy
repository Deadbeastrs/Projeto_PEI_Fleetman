move_forward = { drone, distance, deg ->
    arm drone
    takeoff drone
    turn drone, deg
    move drone, forward: distance
}

gs = [id: "groundStation", position: [lat: 40.635151, lon: -8.659947], ip: '10.1.1.4', mac: 'b4:6b:fc:48:80:19']
enable 'new_relay', rate: 500.ms, groundstation: gs, simulation: false
drone01 = assign 'drone01'
arm drone01
takeoff drone01


coords = [
        // Move at a speed of 12m/s, no yaw is provided so the drone will face the waypoint
        [lat: 40.635154, lon: -8.659789],
        [lat: 40.635205, lon: -8.659584],
        [lat: 40.635147, lon: -8.659525],
        [lat: 40.635205, lon: -8.659584],
        [lat: 40.635154, lon: -8.659789]
]

// [lat: 40.635151, lon: -8.659947]

coords.each { coord ->
    // Send the map and append the takeoff altitude
    move drone01, lat: coord.lat, lon: coord.lon, speed: 1.m/s
    sleep(5000)
}

home drone01
