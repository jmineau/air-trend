# Ben Fasoli
source('../_libraries.r')
source('../_style_setup.r')

body <- dashboardBody(
  includeStyle,
  div(
    class = 'tall',
    fluidRow(
      column(12,
             fluidRow(
               column(3, valueBoxOutput('value_O3_ppb', width = NULL)),
               column(3, valueBoxOutput('value_O3_stabil_ppb', width = NULL)),
               column(3, valueBoxOutput('value_O3_flow_ccm', width = NULL)),
               column(3, valueBoxOutput('value_O3_pres_inhg', width = NULL))
             ),
             box(width = NULL, status = 'danger',
                 div(class = 'tall',
                     plotlyOutput('ts', height = '100%')
                 )
             )
      )
    )
  )
)

dashboardPage(title = 'Teledyne T400', skin = 'black',
              header, sidebar, body)