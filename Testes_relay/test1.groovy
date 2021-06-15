move_forward = { drone, distance, deg ->
    arm drone
    takeoff drone
    turn drone, deg
    move drone, forward: distance
}

gs = [id: "groundStation", position: [lat: 40.6350894, lon: -8.6598095], ip: '10.1.1.4', mac: 'b4:6b:fc:48:80:19']
enable 'new_relay', rate: 500.ms, groundstation: gs, simulation: false
drone01 = assign 'drone02'
arm drone01
takeoff drone01, 10.m


move drone01, forward: 10.m, speed:1.m/s

sleep(10000)

move drone01, lat: 40.6351517, lon: -8.6598169, speed: 1.m/s

home drone01
