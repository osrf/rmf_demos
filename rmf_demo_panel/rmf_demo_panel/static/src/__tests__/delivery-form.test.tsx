import React from 'react';
import { render, screen, fireEvent, within } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import DeliveryForm from '../components/forms/delivery-form';

describe('Delivery Form', () => {
    let root: ReturnType<typeof renderForm>;
    let submitRequest = jest.fn();
    
    function renderForm() {
        const deliveryOptions = {
            "mop": {
              "pickup_place_name": "mopcart_pickup",
              "pickup_dispenser": "mopcart_dispenser",
              "dropoff_place_name": "spill",
              "dropoff_ingestor": "mopcart_collector"
            }
        }
        return render(<DeliveryForm deliveryOptions={deliveryOptions} submitRequest={submitRequest}/>);
    }

    beforeEach(() => {
        root = renderForm();
    });

    test("should render", () => {
        expect(screen.getByText("Schedule a Delivery Request")).toBeInTheDocument();
    });

    test("invalid form will render error message", () => {
        const submitButton = screen.getByText('Submit Request');
        fireEvent.click(submitButton);
        expect(root.container.querySelector('.MuiFormHelperText-root.Mui-error')).toBeTruthy();
    });

    test("submitting a valid form renders success message", () => {
        const submitButton = screen.getByText('Submit Request');
        userEvent.click(root.getByLabelText('Select delivery task'));
        userEvent.click(within(screen.getAllByRole('listbox')[0]).getByText('mop'));
        fireEvent.click(submitButton);
        expect(submitRequest).toHaveBeenCalled();
    });
});
