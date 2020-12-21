const dashboardConfig = {
  valid_task: [
    'Delivery',
    'Clean',
    'Loop'
  ],
  task: {
    Delivery: {
      option: {
        coke: {
          pickup_place_name: 'pantry',
          pickup_dispenser: 'coke_dispenser',
          dropoff_place_name: 'hardware_2',
          dropoff_ingestor: 'coke_ingestor'
        },
        mop: {
          pickup_place_name: 'mopcart_pickup',
          pickup_dispenser: 'mopcart_dispenser',
          dropoff_place_name: 'spill',
          dropoff_ingestor: 'mopcart_collector'
        }
      }
    },
    Loop: {
      places: [
        'coe',
        'pantry'
      ]
    },
    Clean: {
      option: [
        'zone_1',
        'zone_2',
        'zone_3',
        'zone_4'
      ]
    },
    Station: {},
    Patrol: {},
    Charging: {}
  }
}

export default dashboardConfig;