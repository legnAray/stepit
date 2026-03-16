# field_base

StepIt plugin providing foundational field registry, generic field operators and data loading.

### Provided Interfaces

- `stepit::field::DataLoader`: abstract keyed `NdArray` loader.
- `stepit::field::Operator`: abstract field-processing unit.

### Provided Factories

- `stepit::field::DataLoader`:
  - `csv`: loads keyed arrays from a directory of `.csv` files.

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

### Executables

- `data_info`: prints key/shape/dtype summary for arrays loaded through `stepit::field::DataLoader`.

	```shell
	data_info [-f csv/npz/pkl] <data_path> [-- <plugin_args...>]
	```
