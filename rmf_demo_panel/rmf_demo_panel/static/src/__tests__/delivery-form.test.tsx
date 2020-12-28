import React from 'react';
import { render, screen } from '@testing-library/react';
import DeliveryForm from '../components/forms/delivery-form';

describe('Delivery Form', () => {
    const deliveryOptions = {
        "mop": {
          "pickup_place_name": "mopcart_pickup",
          "pickup_dispenser": "mopcart_dispenser",
          "dropoff_place_name": "spill",
          "dropoff_ingestor": "mopcart_collector"
        }
    }
    test("should render", () => {
        render(<DeliveryForm deliveryOptions={deliveryOptions} />);
        expect(screen.getByText("Schedule a Delivery Request")).toBeInTheDocument();
    });
});
