import React from 'react';
import { render, screen } from '@testing-library/react';

import CleaningForm from '../components/forms/cleaning-form';

describe('Footer', () => {
    const cleaningZones = ['zone1', 'zone2'];
    test("should render", () => {
        render(<CleaningForm cleaningZones={cleaningZones} />);
    
        expect(screen.getByRole('cleaning-form')).toBeInTheDocument();
    });
});
