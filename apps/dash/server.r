# Ben Fasoli
source('../_libraries.r')
source('../_constants.r')

plot_build <- function(df, meas) {
  plot_ly(data = df, x = Time, y = df[[meas]],
          hoverinfo = 'x+y', fill = 'tozeroy') %>%
    layout(
      xaxis = list(title = ''),
      yaxis = list(title = meas)
    )
}

function(input, output, session) {
  source('../_reader.r', local = T)
  
  # Value Boxes ----------------------------------------------------------------
  output$value_PM25_ugm3 <- renderValueBox({
    reader[['PM25_ugm3']]() %>%
      tail(1) %>%
      .$PM25_ugm3 %>%
      round(1) %>%
      paste('&#956g m<sup>-3</sup>') %>%
      HTML() %>%
      valueBox(subtitle = HTML('Particulate Matter (PM<sub>2.5</sub>)'),
               color = 'orange', icon = icon('car'), href = '/dash/')
  })
  
  output$value_O3_ppb <- renderValueBox({
    reader[['O3_ppb']]() %>%
      tail(1) %>%
      .$O3_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Ozone (O<sub>3</sub>)'),
               color = 'blue', icon = icon('sun-o'), href = '/teledyne-t400/')
  })
  
  
  # Timeseries -----------------------------------------------------------------
  output$ts <- renderPlotly({
    meas <- 'O3_ppb'
    df <- reader[[meas]]()
    plot_build(df, meas)
    
    O3_ppb <- reader[['O3_ppb']]()
    PM25_ugm3 <- reader[['PM25_ugm3']]()
    
    plot_ly(data = O3_ppb, x = Time, y = O3_ppb, yaxis = 'y1',
            hoverinfo = 'x+y', fill = 'tozeroy') %>%
      add_trace(data = PM25_ugm3, x = Time, y = PM25_ugm3, yaxis = 'y2',
                hoverinfo = 'x+y', fill = 'tozeroy') %>%
      layout(
        showlegend = FALSE,
        xaxis = list(title = '', showgrid = F),
        yaxis = list(anchor = 'x', domain = c(0, 0.49), title = 'Ozone [ppb]'),
        yaxis2 = list(anchor = 'x', domain = c(0.51, 1), title = 'PM2.5 [ugm-3]')
      )
  })
  
}
