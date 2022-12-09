import React from "react"
import { bisect } from "./bisect"

export function DataTable(plots, timestamp_micros) {
    if (plots.length == 0) return null

    const headerRow = (
      <thead>
        <tr><th>Key</th><th>Value</th></tr>
      </thead>
    )

    const dataRows = (
      <tbody>
        {plots.map((plot, plotIndex) => {
          const index = bisect(plot._utime_, (_utime_) => {
            return timestamp_micros - _utime_
          })?.[0]

          const value = plot.series_y[index]
          const enumDescription = plot.hovertext?.[value]

          var valueString = ""
          if (enumDescription != null) {
            valueString = `${enumDescription} (${value})`
          }
          else {
            valueString = value?.toPrecision(6) ?? "-"
          }

          return <tr key={plot.title + plotIndex}>
            <td className="dataKey">{plot.title}</td>
            <td>{valueString}</td>
          </tr>
        })}
      </tbody>
    )

    return (
      <div className="dataTable">
        <table>
          {headerRow}
          {dataRows}
        </table>
      </div>
    )
  }
