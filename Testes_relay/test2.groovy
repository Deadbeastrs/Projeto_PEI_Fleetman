move_forward = { drone, distance, deg ->
    arm drone
    takeoff drone
    turn drone, deg
    move drone, forward: distance
}

gs = [id: "groundStation", position: [lat: 40.634675, lon: -8.660091], ip: '10.1.1.4', mac: 'b4:6b:fc:48:80:19']
//enable 'new_relay', rate: 500.ms, groundstation: gs, simulation: false
drone01 = assign 'drone01'
arm drone01
takeoff drone01

move drone01, forward: 10.m, speed:1.m/s

sleep(5000)

home drone01
