import React, { useState } from 'react';

type NumberInputProps = {
  className?: string;
  onChange?: (value: number) => void;
  placeholder?: string;
};

const NumberInput: React.FC<NumberInputProps> = ({ className, onChange, placeholder }) => {
  const [value, setValue] = useState(0);

  const increment = () => {
    setValue(value + 1);
    if (onChange) {
      onChange(value + 1);
    }
  };

  const decrement = () => {
      setValue(value - 1);
      if (onChange) {
        onChange(value - 1);
      }
  };

  const onChangeHandler = (e: React.ChangeEvent<HTMLInputElement>) => {
    const newValue = Number(e.target.value);
    setValue(newValue);
    if (onChange) {
      onChange(newValue);
    }
  }

  return (
    <div className={`flex w-full ${className}`}>
      <div className="flex flex-col p-2">
        <button onClick={increment} className="text-xs bg-gray-200 hover:bg-gray-300 text-gray-800 font-bold py-0.5 px-1 rounded-t">
          ▲
        </button>
        <button onClick={decrement} className="text-xs bg-gray-200 hover:bg-gray-300 text-gray-800 font-bold py-0.5 px-1 rounded-b">
          ▼
        </button>
      </div>
      <input
        type="text"
        value={value}
        onChange={(e) => {
            const newValue = Number(e.target.value);
            setValue(newValue);
            if (onChange) {
            onChange(newValue);
            }
        }}
        className="w-full text-center text-black"
        placeholder={placeholder}
        />
    </div>
  );
};

export default NumberInput;
