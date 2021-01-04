import React from 'react';
import { fireEvent, render, screen, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event'; 
import CleaningForm from '../components/forms/cleaning-form';

describe('Cleaning Form', () => {
    let root: ReturnType<typeof renderForm>;
    let submitRequest = jest.fn();

    function renderForm() {
        const cleaningZones = ['zone1', 'zone2'];
        return render(<CleaningForm cleaningZones={cleaningZones} submitRequest={submitRequest} />);
    }

    beforeEach(() => {
        root = renderForm();
    });

    test("should render", () => {
        expect(screen.getByRole('cleaning-form')).toBeInTheDocument();
    });

    test("invalid form will render error message", () => {
        const submitButton = screen.getByText('Submit Request');
        fireEvent.click(submitButton);
        expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
        expect(screen.getByText("Cleaning zone cannot be an empty field")).toBeInTheDocument();
    });

    test("submitting a valid form renders success message", () => {
        const submitButton = screen.getByText('Submit Request');
        userEvent.click(root.getByLabelText('Pick a zone'));
        userEvent.click(within(screen.getAllByRole('listbox')[0]).getByText('zone1'));
        fireEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });
});
