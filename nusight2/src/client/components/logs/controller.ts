import { LogsModel } from "./model";

export class LogsController {
  private model: LogsModel;

  constructor(model: LogsModel) {
    this.model = model;
  }

  static of(model: LogsModel): LogsController {
    return new LogsController(model);
  }
}
