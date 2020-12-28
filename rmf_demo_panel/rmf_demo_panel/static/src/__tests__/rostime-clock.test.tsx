import React from 'react';
import { render, screen, act } from '@testing-library/react';
import RostimeClock from '../components/fixed-components/rostime-clock';

describe('Rostime Clock', () => {
    test("should render", async () => {
        await act(async () => {
            render(<RostimeClock />);
        });
        expect(screen.getByRole('clock-time')).toBeInTheDocument();
    });
})