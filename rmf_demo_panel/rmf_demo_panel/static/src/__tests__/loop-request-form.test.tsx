import React from 'react';
import { render, screen } from '@testing-library/react';
import LoopRequestForm from '../components/forms/loop-request-form';

describe('Footer', () => {
    const availablePlaces = ['place1', 'place2'];
    test("should render", () => {
        render(<LoopRequestForm availablePlaces={availablePlaces} />);
        expect(screen.getByText("Schedule a Loop Request")).toBeInTheDocument();
    });
});
