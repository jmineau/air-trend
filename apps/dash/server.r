# Ben Fasoli
source('../_global.r')

function(input, output, session) {
  source('../_reader.r', local = T)

  # Value Boxes ----------------------------------------------------------------
  output$value_1 <- renderValueBox({
    reader[['metone-es642']]() %>%
      tail(1) %>%
      .$PM25_ugm3 %>%
      round(1) %>%
      paste('ug m<sup>-3</sup>') %>%
      HTML() %>%
      valueBox(subtitle = HTML('Particulate Matter (PM<sub>2.5</sub>)'),
               color = 'red', icon = icon('car'), href = '/metone-es642/')
  })

  output$value_2 <- renderValueBox({
    reader[['teledyne-t400']]() %>%
      tail(1) %>%
      .$O3_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Ozone (O<sub>3</sub>)'),
               color = 'green', icon = icon('sun-o'), href = '/teledyne-t400/')
  })

  output$value_3 <- renderValueBox({
    reader[['teledyne-t300']]() %>%
      tail(1) %>%
      .$CO_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Carbon Monoxide (CO)'),
               color = 'yellow', icon = icon('cloud'), href = '/teledyne-t300/')
  })

  output$value_4 <- renderValueBox({
    reader[['teledyne-t200']]() %>%
      tail(1) %>%
      .$NOX_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Nitrous Oxides (NO<sub>x</sub>)'),
               color = 'blue', icon = icon('industry'), href = '/teledyne-t200/')
  })
  
  output$value_5 <- renderValueBox({
    reader[['teledyne-t200']]() %>%
      tail(1) %>%
      .$NO_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Nitrogen Monoxide (NO)'),
               color = 'blue', icon = icon('industry'), href = '/teledyne-t200/')
  })
  
  output$value_6 <- renderValueBox({
    reader[['teledyne-t200']]() %>%
      tail(1) %>%
      .$NO2_ppb %>%
      round(2) %>%
      paste('ppb') %>%
      valueBox(subtitle = HTML('Nitrogen Dioxide (NO<sub>2</sub>)'),
               color = 'blue', icon = icon('industry'), href = '/teledyne-t200/')
  })


  # Timeseries -----------------------------------------------------------------
  output$ts <- renderPlotly({
    df <- bind_rows(reader[['metone-es642']](),
                    reader[['teledyne-t400']](),
                    reader[['teledyne-t300']](),
                    reader[['teledyne-t200']]()) %>%
      select(Time, PM25_ugm3, O3_ppb, CO_ppb, NOX_ppb, NO_ppb, NO2_ppb)

    make_subplot(df) %>%
      layout(yaxis = list(title = 'PM2.5\n[ugm-3]'),
             yaxis2 = list(title = 'O3\n[ppb]'),
             yaxis3 = list(title = 'CO\n[ppb]'),
             yaxis4 = list(title = 'NOx\n[ppb]'),
             yaxis5 = list(title = 'NO\n[ppb]'),
             yaxis6 = list(title = 'NO2\n[ppb]')
      )
  })
}
