import { useCallback, useMemo, useState } from "react";
import React from "react";
import { Icon } from "@components/icon/view";
import { NbsScrubberModel } from "@components/nbs_scrubbers/model";
import { NbsScrubberController } from "@components/nbs_scrubbers/nbs_scrubber/controller";
import { ResizeContainer } from "@components/resize_container/resize_container";
import { ResizePanel } from "@components/resize_container/resize_panel";
import { TimelineHeader } from "@components/timeline/header/timeline_header";
import { TimelineMarker } from "@components/timeline/marker/timeline_marker";
import { TimelineCameraView, TimelineDataPoint } from "@components/timeline/model";
import { PreventMouseScroll } from "@components/timeline/prevent_mouse_scroll";
import { TimelineRow } from "@components/timeline/row/timeline_row";
import { TimelineView } from "@components/timeline/view";
import { useRpcController } from "@hooks/use_rpc_controller";
import { useUpdatable } from "@hooks/use_updatable";
import classNames from "classnames";
import { observer } from "mobx-react";

import { MessageTypeId, TypeIndex } from "../model";

import { PacketTimelineController } from "./controller";
import { MessageView } from "./message/view";
import { getMessageColor, messageNameToIcon, PacketTimelineModel } from "./model";

export interface PacketTimelineProps {
  scrubber: NbsScrubberModel;
  indices: TypeIndex[];
  controller: NbsScrubberController;
  timelineCameraView: TimelineCameraView;
}

export const PacketTimeline = observer((props: PacketTimelineProps) => {
  const { scrubber, indices, controller, timelineCameraView } = props;

  const model = useUpdatable(
    () => new PacketTimelineModel(scrubber, timelineCameraView),
    (instance, [scrubber, view]) => instance.update(scrubber, view),
    [scrubber, timelineCameraView],
  );

  const timelineController = useRpcController(
    (network) => new PacketTimelineController(network, model, controller),
    [scrubber, model],
  );

  return (
    <div className="flex w-full h-full">
      <ResizeContainer horizontal>
        <ResizePanel minSize={300} initialRatio={0.75}>
          <TimelineView view={timelineCameraView} className="flex flex-col w-full h-full">
            <div className="h-0 grow z-0 pointer-events-none">
              <div className="w-full h-full flex flex-col overflow-y-auto">
                <PacketTimelineHeader model={model} controller={timelineController} />
                <PacketTimelineBody indices={indices} model={model} controller={timelineController} />
              </div>
            </div>
          </TimelineView>
        </ResizePanel>
        <ResizePanel minSize={300} initialRatio={0.25} className="bg-auto-surface-1 whitespace-pre-wrap text-left">
          {model.selectedMessage ? (
            <MessageView viewModel={model.selectedMessage} />
          ) : (
            <div className="flex flex-col gap-2 items-center justify-center h-full">
              <Icon className="text-auto-icon">info</Icon>
              <span className="text-auto-hint text-sm">No packet selected</span>
            </div>
          )}
        </ResizePanel>
      </ResizeContainer>
    </div>
  );
});

const PacketTimelineHeader = observer(function PacketTimelineHeader(props: {
  model: PacketTimelineModel;
  controller: PacketTimelineController;
}) {
  const { model, controller } = props;
  const [isScrubbing, setIsScrubbing] = useState(false);
  const [hoveredTime, setHoveredTime] = useState<bigint | undefined>(undefined);

  const startScrubbing = useCallback(
    (timestamp: bigint) => {
      controller.seek(timestamp);
      setIsScrubbing(true);
    },
    [controller, setIsScrubbing],
  );

  const onScrub = useCallback(
    (timestamp: bigint) => {
      if (isScrubbing) {
        controller.seek(timestamp);
      }
      setHoveredTime(timestamp);
    },
    [isScrubbing, setHoveredTime],
  );

  const stopScrubbing = useCallback(() => setIsScrubbing(false), [setIsScrubbing]);

  return (
    <>
      <div className="flex w-full h-12 border-b border-auto shrink-0 sticky top-0 bg-auto-surface-2 pointer-events-auto">
        <div className="w-80 bg-auto-surface-2 border-r border-auto"></div>
        <PreventMouseScroll className="flex-1">
          <TimelineHeader
            className="w-full h-full"
            onPointerDown={startScrubbing}
            onPointerMove={onScrub}
            onPointerUp={stopScrubbing}
            onMouseLeave={() => setHoveredTime(undefined)}
          />
        </PreventMouseScroll>
      </div>
      <TimelineMarker timestamp={model.scrubber.current} color="var(--color-nusight-500)" clampToView />
      {!isScrubbing && hoveredTime !== undefined ? (
        <TimelineMarker timestamp={hoveredTime} color="var(--color-auto-contrast-2)" />
      ) : null}
    </>
  );
});

const PacketTimelineBody = observer(function PacketTimelineBody(props: {
  indices: TypeIndex[];
  model: PacketTimelineModel;
  controller: PacketTimelineController;
}) {
  const { indices, model, controller } = props;

  return (
    <>
      {indices.map((typeIndex) => (
        <PacketTimelineBodyRow
          key={typeIndex.type.typeHash + typeIndex.type.subtype}
          typeIndex={typeIndex}
          model={model}
          controller={controller}
        />
      ))}
      {/* Render a final blank row to fill the remaining height */}
      <div className="flex w-full h-full -z-[1] pointer-events-auto">
        <div className="w-80 bg-auto-surface-2 border-r border-auto"></div>
        <div className="flex-1">
          <TimelineRow className="w-full h-full" />
        </div>
      </div>
    </>
  );
});

const PacketTimelineBodyRow = observer(function PacketTimelineBodyRow(props: {
  typeIndex: TypeIndex;
  model: PacketTimelineModel;
  controller: PacketTimelineController;
}) {
  const { typeIndex, model, controller } = props;
  const type = typeIndex.type;
  const selected = model.selectedMessage;
  const isSelected = type.typeHash === selected?.type.typeHash && type.subtype === selected?.type.subtype;

  const color = useMemo(() => {
    const nameOrHash = type.typeName === "" ? type.typeHash : type.typeName;
    return getMessageColor(nameOrHash);
  }, [type.typeName]);

  const dataPoints = useMemo<TimelineDataPoint[]>(
    () => typeIndex.timestamps.map((timestamp) => ({ timestamp, color })),
    [typeIndex.timestamps],
  );

  const onDataPointClick = useCallback(
    (dataPoint: TimelineDataPoint) => {
      if (model.highlightedDataPoints.includes(dataPoint)) {
        controller.deselectPacket();
      } else {
        const index = typeIndex.timestamps.indexOf(dataPoint.timestamp);
        controller.selectPacket(type, index, dataPoint);
      }
    },
    [typeIndex, controller],
  );

  return (
    <div className="relative flex w-full h-8 shrink-0 border-b border-auto -z-[1] pointer-events-auto">
      <div className="w-80 bg-auto-surface-2 border-r border-auto">
        <PacketTypeView type={type} isSelected={isSelected} />
      </div>
      <div className="flex-1">
        <PreventMouseScroll className="w-full h-full">
          <TimelineRow
            onDataPointClick={onDataPointClick}
            dataPoints={dataPoints}
            highlightedDataPoints={model.highlightedDataPoints}
            dataPointHeight={16}
            dataPointWidth={12}
            className={classNames("w-full h-full", isSelected ? "bg-nusight-500/20" : "")}
          />
        </PreventMouseScroll>
      </div>
    </div>
  );
});

function PacketTypeView(props: { type: MessageTypeId; isSelected: boolean }) {
  const { type, isSelected } = props;
  const { typeName, typeHash, subtype } = type;

  // Empty name indicates that the name is not known
  const fullName = typeName === "" ? "Unknown" : typeName;
  const baseName = fullName.split(".").at(-1);

  // 0 subtype indicates that the message has no subtype
  const hasSubtype = subtype !== 0;
  const fullNameWithSubtype = fullName + (hasSubtype ? "#" + subtype : "");

  const iconName = messageNameToIcon[fullName] ?? messageNameToIcon["default"];

  return (
    <div className={classNames("h-full", isSelected ? "dark" : "")} title={fullNameWithSubtype}>
      <div
        className={classNames("flex h-full px-2 gap-2 items-center", isSelected ? "bg-nusight-500" : "bg-transparent")}
      >
        <Icon size="20" className="text-auto-icon">
          {iconName}
        </Icon>
        <div className="flex gap-1 truncate text-sm text-auto-primary">
          {baseName}
          {typeName === "" ? <span className="text-auto-secondary">({typeHash})</span> : null}
          {hasSubtype ? <span className="text-auto-secondary">#{subtype}</span> : null}
        </div>
      </div>
    </div>
  );
}
