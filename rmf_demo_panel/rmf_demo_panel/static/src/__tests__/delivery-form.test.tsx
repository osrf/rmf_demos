import React from 'react';
import { render, screen } from '@testing-library/react';
import DeliveryForm from '../components/forms/loop-request-form';

describe('Delivery Form', () => {
    const availablePlaces = ['place1', 'place2'];
    test("should render", () => {
        render(<DeliveryForm availablePlaces={availablePlaces} />);
    
        expect(screen.getByText("Schedule a Delivery Request")).toBeInTheDocument();
    });
});
