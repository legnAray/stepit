# field_base

StepIt plugin providing foundational field registry and generic field operators.

### Provided Interfaces

- `stepit::field::Operator`: abstract field-processing unit.

### Provided Factories

- `stepit::field::Operator`:

  | Name          | Description                                                                 |
  | :------------ | :-------------------------------------------------------------------------- |
  | `affine`      | Applies element-wise affine transform (`x * scale + bias`) to a field.     |
  | `concat`      | Concatenates multiple source fields into one target field.                  |
  | `const`       | Writes a constant scalar/vector value into a target field.                  |
  | `copy`        | Copies one source field to another target field.                            |
  | `history`     | Stacks temporal history of a field into one output vector.                  |
  | `masked_fill` | Fills selected indices/ranges of a field with a constant value.             |
  | `slice`       | Extracts selected indices/ranges from a source field into a target field.   |
  | `split`       | Splits one source field into multiple target fields by configured segments. |
